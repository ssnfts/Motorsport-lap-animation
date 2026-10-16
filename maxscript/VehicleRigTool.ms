/*
    VehicleRigTool.ms
    Native 3ds Max vehicle rig and path motion tool.

    Binds one existing vehicle and one open spline, validates the path, and
    bakes editable keys onto generated VRIG_ helpers. Source geometry and
    existing controllers are left intact; every bound node records its
    original parent and transform so the rig can be removed cleanly.

    The vehicle is measured from its evaluated meshes (wheel centres, tyre
    radius, ground plane, wheelbase and facing), never from object pivots.
    Both axles stand on the path, so heading and slope follow the road, and
    every rotation is written to its Euler track unwrapped.

    Parameter names were verified on 3ds Max 2027. Treat other versions as
    unverified until checked on the host.

    Out of scope by design: file reads and writes, network access, terrain,
    lighting, cameras, and rendering.
*/

struct VehicleRigProfile
(
    name, wheelbase, track, tyreRadius, topSpeed, acceleration, braking,
    lateralGrip, maxSteer, springTravel, dampingRatio, maxRoll, maxPitch,
    rollGradient, squatGradient, diveGradient, rideFrequency, understeer, steeringRatio
)

struct VehicleRigIssue (code, severity, message, index)

-- A sampled path in metres, with the end directions used to stand an
-- axle beyond either end.
struct VehicleRigPath (points, distances, startDir, endDir, length)

-- slopes are nose-up degrees; verticalCurvatures are positive in a dip.
struct VehicleRigMotion
(
    path, yaws, curvatures, slopes, verticalCurvatures, speeds, times, duration
)

-- Measured in scene units from the vehicle's evaluated meshes.
struct VehicleRigMeasure
(
    wheelCentres, tyreRadius, groundZ, groundCentre, wheelbase, track,
    forwardIndex, bodyCentre, axleZ, length
)

-- How far the body may move before the model's own wheel arches meet the
-- tyres. compression and archGaps are per wheel in scene units; the turn
-- limits are the steering angles each front wheel can reach, as
-- #(inner, outer) for a left and a right turn.
struct VehicleRigClearance
(
    compression, archGaps, leftTurnLimits, rightTurnLimits, ignored, seconds
)

-- forwardIndex left undefined takes the facing measured from the wheels
-- at rest; set it only to override that measurement. gripUsage is the
-- share of the tyres' grip the driver uses.
struct VehicleRigBakeSettings
(
    profile, splineNode, rootNode, bodyNode, wheelNodes, steeringWheelNode,
    startFrame = 0, endFrame = 100, reversed = false, targetSpeed = 30.0,
    rollIntensity = 1.0, accelIntensity = 1.0, brakeIntensity = 1.0, suspIntensity = 1.0,
    sampleCount = 256, scale = 1.0, forwardIndex = undefined, matchDriveTime = true,
    gripUsage = 0.55, settleSeconds = 1.2
)

/* ------------------------------------------------------------------ *
 * Vehicle presets. These mirror server/vehicle_dynamics.py exactly;
 * tests/test_vehicle_rig_script.py fails if the two ever drift.
 * ------------------------------------------------------------------ */

fn vrigPresets =
(
    #(
        VehicleRigProfile name:"car" wheelbase:2.70 track:1.55 tyreRadius:0.31 topSpeed:70.0 acceleration:5.5 braking:9.0 lateralGrip:1.00 maxSteer:35.0 springTravel:0.12 dampingRatio:0.70 maxRoll:6.0 maxPitch:4.0 rollGradient:3.5 squatGradient:1.5 diveGradient:2.0 rideFrequency:1.4 understeer:1.5 steeringRatio:15.0,
        VehicleRigProfile name:"suv" wheelbase:2.85 track:1.65 tyreRadius:0.36 topSpeed:55.0 acceleration:4.0 braking:8.0 lateralGrip:0.85 maxSteer:32.0 springTravel:0.18 dampingRatio:0.75 maxRoll:9.0 maxPitch:5.0 rollGradient:5.5 squatGradient:2.0 diveGradient:2.5 rideFrequency:1.2 understeer:2.5 steeringRatio:16.0,
        VehicleRigProfile name:"pickup" wheelbase:3.20 track:1.72 tyreRadius:0.38 topSpeed:48.0 acceleration:3.2 braking:7.0 lateralGrip:0.75 maxSteer:30.0 springTravel:0.20 dampingRatio:0.80 maxRoll:10.0 maxPitch:5.5 rollGradient:6.0 squatGradient:2.5 diveGradient:3.0 rideFrequency:1.2 understeer:3.0 steeringRatio:18.0,
        VehicleRigProfile name:"van" wheelbase:3.10 track:1.70 tyreRadius:0.35 topSpeed:42.0 acceleration:3.0 braking:6.5 lateralGrip:0.70 maxSteer:28.0 springTravel:0.18 dampingRatio:0.80 maxRoll:10.0 maxPitch:5.0 rollGradient:6.0 squatGradient:2.0 diveGradient:2.5 rideFrequency:1.2 understeer:3.0 steeringRatio:17.0,
        VehicleRigProfile name:"bus" wheelbase:6.00 track:2.35 tyreRadius:0.49 topSpeed:30.0 acceleration:1.5 braking:5.0 lateralGrip:0.55 maxSteer:24.0 springTravel:0.16 dampingRatio:0.85 maxRoll:7.0 maxPitch:3.5 rollGradient:7.0 squatGradient:1.5 diveGradient:2.0 rideFrequency:1.0 understeer:3.5 steeringRatio:20.0
    )
)

fn vrigPresetNames =
(
    for profile in (vrigPresets()) collect profile.name
)

fn vrigPresetByName presetName =
(
    local found = undefined
    for profile in (vrigPresets()) where profile.name == presetName do found = profile
    found
)

fn vrigCopyProfile profile =
(
    VehicleRigProfile name:profile.name wheelbase:profile.wheelbase \
        track:profile.track tyreRadius:profile.tyreRadius topSpeed:profile.topSpeed \
        acceleration:profile.acceleration braking:profile.braking \
        lateralGrip:profile.lateralGrip maxSteer:profile.maxSteer \
        springTravel:profile.springTravel dampingRatio:profile.dampingRatio \
        maxRoll:profile.maxRoll maxPitch:profile.maxPitch \
        rollGradient:profile.rollGradient squatGradient:profile.squatGradient \
        diveGradient:profile.diveGradient rideFrequency:profile.rideFrequency \
        understeer:profile.understeer steeringRatio:profile.steeringRatio
)

/* ------------------------------------------------------------------ *
 * Units and numeric helpers
 * ------------------------------------------------------------------ */

fn vrigGravity = 9.80665

-- System units contained in one metre. All physics runs in SI; only the
-- scene transforms are converted.
fn vrigUnitScale =
(
    local scale = units.decodeValue "1m"
    if scale == undefined or scale <= 0.0 then 1.0 else scale
)

fn vrigIsFinite value =
(
    local ok = true
    if not (value == value) do ok = false
    if ok and (abs value) > 1e30 do ok = false
    ok
)

fn vrigClamp value limit = ( amax -limit (amin value limit) )

fn vrigSquare value = ( value * value )

/* ------------------------------------------------------------------ *
 * Deterministic vehicle maths, mirroring server/vehicle_dynamics.py
 * ------------------------------------------------------------------ */

fn vrigValidateProfile profile =
(
    local errors = #()
    if profile == undefined then
    (
        append errors "No vehicle profile selected"
        return errors
    )

    local positives = #(
        #("wheelbase", profile.wheelbase), #("track", profile.track),
        #("tyre radius", profile.tyreRadius), #("top speed", profile.topSpeed),
        #("acceleration", profile.acceleration), #("braking", profile.braking),
        #("lateral grip", profile.lateralGrip), #("max steer", profile.maxSteer),
        #("spring travel", profile.springTravel), #("max roll", profile.maxRoll),
        #("max pitch", profile.maxPitch), #("roll gradient", profile.rollGradient),
        #("squat gradient", profile.squatGradient), #("dive gradient", profile.diveGradient),
        #("ride frequency", profile.rideFrequency), #("steering ratio", profile.steeringRatio)
    )
    for entry in positives do
    (
        if not (vrigIsFinite entry[2]) then
            append errors (entry[1] + " must be a finite number")
        else if entry[2] <= 0.0 do
            append errors (entry[1] + " must be greater than zero")
    )
    if not (vrigIsFinite profile.dampingRatio) then
        append errors "damping ratio must be a finite number"
    else if profile.dampingRatio < 0.0 or profile.dampingRatio > 1.0 do
        append errors "damping ratio must be between zero and one"
    if not (vrigIsFinite profile.understeer) then
        append errors "understeer must be a finite number"
    else if profile.understeer < 0.0 do
        append errors "understeer must not be negative"
    errors
)

fn vrigCurveSpeedLimit radius profile =
(
    amin profile.topSpeed (sqrt ((amax radius 0.0) * (vrigGravity()) * profile.lateralGrip))
)

-- MaxScript trigonometry works in degrees, so no radian conversion here.
fn vrigAckermannAngles steeringDeg wheelbase track maxSteer =
(
    if steeringDeg == 0.0 then #(0.0, 0.0)
    else
    (
        local sign = if steeringDeg < 0.0 then -1.0 else 1.0
        local centreAngle = amin (abs steeringDeg) maxSteer
        local turnRadius = wheelbase / (tan centreAngle)
        local innerRadius = amax (turnRadius - track / 2.0) 0.0
        local outerRadius = turnRadius + track / 2.0
        local inner = if innerRadius > 0.0 then (atan (wheelbase / innerRadius)) else 90.0
        local outer = atan (wheelbase / outerRadius)
        #(sign * (amin inner maxSteer), sign * (amin outer maxSteer))
    )
)

-- Centre front-wheel angle for a turn, left positive; mirrors
-- vehicle_dynamics.steering_angle_deg. Understeer adds lock as the
-- cornering load rises, so a fast corner shows more than a slow one.
fn vrigSteeringAngle curvature speed profile =
(
    local geometric = atan (profile.wheelbase * curvature)
    local extra = profile.understeer * speed * speed * (abs curvature) / (vrigGravity())
    if curvature < 0.0 do extra = -extra
    vrigClamp (geometric + extra) profile.maxSteer
)

fn vrigAckermannFits angle profile innerLimit outerLimit =
(
    local angles = vrigAckermannAngles angle profile.wheelbase profile.track 90.0
    angles[1] <= innerLimit and angles[2] <= outerLimit
)

-- Largest centre angle whose Ackermann inner and outer wheel angles stay
-- within the clearance; mirrors vehicle_dynamics.max_centre_steer_deg.
fn vrigMaxCentreSteer profile innerLimit outerLimit =
(
    local ceiling = profile.maxSteer
    if innerLimit <= 0.0 or outerLimit <= 0.0 then 0.0
    else if vrigAckermannFits ceiling profile innerLimit outerLimit then ceiling
    else
    (
        local low = 0.0
        local high = ceiling
        for iteration = 1 to 40 do
        (
            local middle = (low + high) / 2.0
            if vrigAckermannFits middle profile innerLimit outerLimit then low = middle else high = middle
        )
        low
    )
)

fn vrigWheelRotationDegrees distance tyreRadius =
(
    if tyreRadius <= 0.0 then 0.0 else radToDeg (distance / tyreRadius)
)

-- The pose the chassis settles to; mirrors vehicle_dynamics.body_response.
-- Roll and pitch follow per-g gradients. Weight transfer moves load between
-- wheels rather than adding to it, so heave comes only from `vertical`,
-- the road's upward acceleration through a dip (positive) or over a crest.
-- Returns #(roll, pitch, heave): roll negative leans right, pitch is
-- nose-up, heave in metres.
fn vrigBodyResponse profile longitudinal lateral vertical rollIntensity accelIntensity brakeIntensity suspIntensity =
(
    local g = vrigGravity()
    local roll = vrigClamp (-lateral / g * profile.rollGradient * rollIntensity) profile.maxRoll
    local gradient = if longitudinal >= 0.0 then profile.squatGradient else profile.diveGradient
    local share = if longitudinal >= 0.0 then accelIntensity else brakeIntensity
    local pitch = vrigClamp (longitudinal / g * gradient * share) profile.maxPitch
    local omega = 2.0 * pi * profile.rideFrequency
    local heave = vrigClamp (-vertical / (omega * omega) * suspIntensity) profile.springTravel
    #(roll, pitch, heave)
)

/* ------------------------------------------------------------------ *
 * Path preflight, mirroring server/spline_preflight.py
 * ------------------------------------------------------------------ */

fn vrigOrientation startPoint endPoint testPoint =
(
    (endPoint.x - startPoint.x) * (testPoint.y - startPoint.y) - \
    (endPoint.y - startPoint.y) * (testPoint.x - startPoint.x)
)

fn vrigPointOnSegment testPoint startPoint endPoint epsilon =
(
    (amin startPoint.x endPoint.x) - epsilon <= testPoint.x and \
    testPoint.x <= (amax startPoint.x endPoint.x) + epsilon and \
    (amin startPoint.y endPoint.y) - epsilon <= testPoint.y and \
    testPoint.y <= (amax startPoint.y endPoint.y) + epsilon
)

fn vrigSegmentsIntersect2D firstStart firstEnd secondStart secondEnd =
(
    local epsilon = 1e-9
    local a = vrigOrientation firstStart firstEnd secondStart
    local b = vrigOrientation firstStart firstEnd secondEnd
    local c = vrigOrientation secondStart secondEnd firstStart
    local d = vrigOrientation secondStart secondEnd firstEnd

    if ((a > epsilon and b < -epsilon) or (a < -epsilon and b > epsilon)) and \
       ((c > epsilon and d < -epsilon) or (c < -epsilon and d > epsilon)) then true
    else
    (
        ((abs a) <= epsilon and (vrigPointOnSegment secondStart firstStart firstEnd epsilon)) or \
        ((abs b) <= epsilon and (vrigPointOnSegment secondEnd firstStart firstEnd epsilon)) or \
        ((abs c) <= epsilon and (vrigPointOnSegment firstStart secondStart secondEnd epsilon)) or \
        ((abs d) <= epsilon and (vrigPointOnSegment firstEnd secondStart secondEnd epsilon))
    )
)

-- Returns #(radius, turnAngleDegrees). A radius of undefined means the three
-- points are collinear or degenerate.
fn vrigCurvatureRadius2D previous current following =
(
    local epsilon = 1e-9
    local inX = current.x - previous.x
    local inY = current.y - previous.y
    local outX = following.x - current.x
    local outY = following.y - current.y
    local inLength = sqrt ((vrigSquare inX) + (vrigSquare inY))
    local outLength = sqrt ((vrigSquare outX) + (vrigSquare outY))
    if inLength <= epsilon or outLength <= epsilon then #(undefined, 0.0)
    else
    (
        local cosine = amax -1.0 (amin 1.0 ((inX * outX + inY * outY) / (inLength * outLength)))
        local turnAngle = acos cosine
        local chord = sqrt ((vrigSquare (following.x - previous.x)) + (vrigSquare (following.y - previous.y)))
        local twiceArea = abs (inX * outY - inY * outX)
        if twiceArea <= epsilon then #(undefined, turnAngle)
        else #(inLength * outLength * chord / (2.0 * twiceArea), turnAngle)
    )
)

fn vrigPreflightOpenPath points profile =
(
    local epsilon = 1e-9
    local sharpKinkDeg = 45.0
    local issues = #()

    for point in points do
    (
        if not ((vrigIsFinite point.x) and (vrigIsFinite point.y) and (vrigIsFinite point.z)) do
        (
            append issues (VehicleRigIssue code:"invalid_coordinate" severity:"error" \
                message:"Path coordinates must be finite" index:undefined)
            return issues
        )
    )

    local distinct = #()
    for point in points do
    (
        local seen = false
        for other in distinct where (distance point other) <= epsilon do seen = true
        if not seen do append distinct point
    )
    if distinct.count < 2 do
        append issues (VehicleRigIssue code:"too_short" severity:"error" \
            message:"Path needs at least two distinct points" index:undefined)

    local zeroLength = #()
    for index = 1 to points.count - 1 do
    (
        if (distance points[index] points[index + 1]) <= epsilon do
        (
            append zeroLength index
            append issues (VehicleRigIssue code:"zero_length_segment" severity:"error" \
                message:"Path contains a zero-length segment" index:index)
            append issues (VehicleRigIssue code:"duplicate_knot" severity:"error" \
                message:"Adjacent path points are duplicates" index:(index + 1))
        )
    )

    for first = 1 to points.count - 1 do
    (
        if (findItem zeroLength first) == 0 do
        (
            for second = first + 2 to points.count - 1 do
            (
                if (findItem zeroLength second) == 0 do
                (
                    if vrigSegmentsIntersect2D points[first] points[first + 1] points[second] points[second + 1] do
                        append issues (VehicleRigIssue code:"self_intersection" severity:"warning" \
                            message:"Path crosses itself in the vehicle travel plane" index:second)
                )
            )
        )
    )

    for index = 2 to points.count - 1 do
    (
        local curvature = vrigCurvatureRadius2D points[index - 1] points[index] points[index + 1]
        local radius = curvature[1]
        local turnAngle = curvature[2]
        if turnAngle >= sharpKinkDeg do
            append issues (VehicleRigIssue code:"sharp_kink" severity:"warning" \
                message:"Path turns abruptly; smooth this knot before baking" index:index)
        if radius != undefined do
        (
            local requiredSteer = atan (profile.wheelbase / radius)
            if requiredSteer > profile.maxSteer do
                append issues (VehicleRigIssue code:"steering_limit" severity:"warning" \
                    message:"Path curvature exceeds the vehicle steering limit" index:index)
        )
    )

    issues
)

fn vrigCountBySeverity issues wanted =
(
    local total = 0
    for issue in issues where issue.severity == wanted do total += 1
    total
)

fn vrigFormatIssues issues =
(
    local lines = #()
    for issue in issues do
    (
        local location = if issue.index == undefined then "" else (" at knot " + (issue.index as string))
        append lines ("[" + issue.severity + "] " + issue.code + location + ": " + issue.message)
    )
    lines
)

/* ------------------------------------------------------------------ *
 * Spline sampling and path queries
 * ------------------------------------------------------------------ */

fn vrigValidateSplineNode splineNode =
(
    local errors = #()
    if splineNode == undefined then append errors "No spline selected"
    else if not (isValidNode splineNode) then append errors "The picked spline no longer exists"
    else if not (isKindOf splineNode SplineShape or isKindOf splineNode Line) then
        append errors "Selected node is not an editable spline"
    else
    (
        if (numSplines splineNode) != 1 do
            append errors ("Path must contain exactly one spline segment; found " + ((numSplines splineNode) as string))
        if (numSplines splineNode) >= 1 do
        (
            if isClosed splineNode 1 do append errors "Path must be an open spline"
            if (numKnots splineNode 1) < 2 do append errors "Path needs at least two knots"
            if (curveLength splineNode 1) <= 0.0 do append errors "Path has zero length"
        )
    )
    errors
)

-- Long paths need more samples to keep corners round; short ones keep the
-- artist's figure. Roughly two samples per metre, capped.
fn vrigSampleCountFor splineNode sampleCount scale =
(
    local metres = (curveLength splineNode 1) / scale
    amin 8192 (amax sampleCount ((ceil (metres * 2.0)) as integer))
)

fn vrigCumulativeDistances points =
(
    local distances = #(0.0)
    for index = 1 to points.count - 1 do
        append distances (distances[index] + (distance points[index] points[index + 1]))
    distances
)

fn vrigBezierPoint p0 p1 p2 p3 t =
(
    local u = 1.0 - t
    p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) + p3 * (t * t * t)
)

-- Samples the spline evenly by arc length and returns world-space points
-- in metres; `scale` is scene units per metre. Each segment is evaluated
-- from its knots and handles rather than through lengthInterp, which on
-- 3ds Max 2027 returns repeated and polyline-snapped points that read as
-- false corners and self-intersections. The last sample is the last knot.
fn vrigSamplePath splineNode sampleCount scale =
(
    local offset = splineNode.objecttransform
    local knots = numKnots splineNode 1
    local perSegment = 64
    local dense = #()
    for knot = 1 to knots - 1 do
    (
        local p0 = getKnotPoint splineNode 1 knot
        local p1 = getOutVec splineNode 1 knot
        local p2 = getInVec splineNode 1 (knot + 1)
        local p3 = getKnotPoint splineNode 1 (knot + 1)
        for step = (if knot == 1 then 0 else 1) to perSegment do
            append dense ((vrigBezierPoint p0 p1 p2 p3 ((step as float) / perSegment)) * offset)
    )
    local lengths = vrigCumulativeDistances dense
    local total = lengths[lengths.count]
    local points = #()
    local cursor = 1
    for index = 0 to sampleCount do
    (
        local wanted = total * (index as float) / (sampleCount as float)
        while cursor < (dense.count - 1) and lengths[cursor + 1] < wanted do cursor += 1
        local span = lengths[cursor + 1] - lengths[cursor]
        local blend = if span <= 1e-12 then 0.0 else (amin 1.0 (amax 0.0 ((wanted - lengths[cursor]) / span)))
        local p = dense[cursor] + blend * (dense[cursor + 1] - dense[cursor])
        append points [p.x / scale, p.y / scale, p.z / scale]
    )
    points
)

fn vrigKnotPoints splineNode scale =
(
    local offset = splineNode.objecttransform
    for index = 1 to (numKnots splineNode 1) collect
    (
        local p = (getKnotPoint splineNode 1 index) * offset
        [p.x / scale, p.y / scale, p.z / scale]
    )
)

-- A sampled path with the end directions needed to stand an axle beyond
-- either end. All values are metres.
fn vrigMakePath points =
(
    local distances = vrigCumulativeDistances points
    local startDir = [1, 0, 0]
    local found = false
    for index = 1 to points.count - 1 while not found do
        if (distance points[index] points[index + 1]) > 1e-9 do
        (
            startDir = normalize (points[index + 1] - points[index])
            found = true
        )
    local endDir = startDir
    found = false
    for index = points.count to 2 by -1 while not found do
        if (distance points[index] points[index - 1]) > 1e-9 do
        (
            endDir = normalize (points[index] - points[index - 1])
            found = true
        )
    VehicleRigPath points:points distances:distances startDir:startDir \
        endDir:endDir length:distances[distances.count]
)

fn vrigReversePath path =
(
    local flipped = for index = path.points.count to 1 by -1 collect path.points[index]
    vrigMakePath flipped
)

-- Point `travelled` metres along the path. Beyond either end the path
-- continues straight along its end direction, mirroring
-- vehicle_dynamics.point_at_distance.
fn vrigPathPoint path travelled =
(
    local points = path.points
    local distances = path.distances
    local count = points.count
    if travelled <= 0.0 then points[1] + path.startDir * travelled
    else if travelled >= path.length then points[count] + path.endDir * (travelled - path.length)
    else
    (
        local low = 1
        local high = count
        while (high - low) > 1 do
        (
            local middle = (low + high) / 2
            if distances[middle] <= travelled then low = middle else high = middle
        )
        local span = distances[high] - distances[low]
        local blend = if span <= 1e-12 then 0.0 else ((travelled - distances[low]) / span)
        points[low] + blend * (points[high] - points[low])
    )
)

-- Both axles stand on the path, half a wheelbase either side of the
-- vehicle centre. Heading and slope come from the line between them, so a
-- corner knot turns the vehicle over its own length and nothing is
-- integrated that could drift. Mirrors vehicle_dynamics.axle_pose.
-- Returns #(contact position, heading degrees, nose-up pitch degrees).
fn vrigAxlePose path travelled wheelbase =
(
    local rear = vrigPathPoint path (travelled - wheelbase / 2.0)
    local front = vrigPathPoint path (travelled + wheelbase / 2.0)
    local axis = front - rear
    local flat = sqrt (axis.x * axis.x + axis.y * axis.y)
    #((front + rear) / 2.0, atan2 axis.y axis.x, atan2 axis.z flat)
)

-- Moves an angle by whole turns so it lies within half a turn of the
-- reference; keyed values then interpolate the short way.
fn vrigUnwrapDegrees value reference =
(
    local result = value
    while (result - reference) > 180.0 do result -= 360.0
    while (result - reference) < -180.0 do result += 360.0
    result
)

/* ------------------------------------------------------------------ *
 * Motion plan: two-pass grip-limited speed profile
 * ------------------------------------------------------------------ */

-- Longitudinal acceleration left over once cornering has taken its share
-- of the grip (the friction circle).
fn vrigLongitudinalAvailable limit speed curvature lateralLimit =
(
    local usage = amin 1.0 (speed * speed * (abs curvature) / lateralLimit)
    limit * (sqrt (amax 0.0 (1.0 - usage * usage)))
)

-- Speeds from rest to rest; mirrors vehicle_dynamics.plan_speed_profile.
-- gripUsage is the share of the tyres' grip the driver uses. Braking and
-- acceleration share it with cornering, and drive tapers with speed.
fn vrigSpeedProfile distances curvatures profile targetSpeed gripUsage =
(
    local count = distances.count
    local lateralLimit = gripUsage * profile.lateralGrip * (vrigGravity())
    local ceiling = amin targetSpeed profile.topSpeed
    local speeds = for curvature in curvatures collect \
        (if (abs curvature) <= 1e-9 then ceiling else (amin ceiling (sqrt (lateralLimit / (abs curvature)))))
    speeds[1] = 0.0
    speeds[count] = 0.0
    for index = 2 to count do
    (
        local speed = speeds[index - 1]
        local drive = gripUsage * profile.acceleration * (amax 0.0 (1.0 - (vrigSquare (speed / profile.topSpeed))))
        local accel = vrigLongitudinalAvailable drive speed curvatures[index - 1] lateralLimit
        local span = distances[index] - distances[index - 1]
        speeds[index] = amin speeds[index] (sqrt (speed * speed + 2.0 * accel * span))
    )
    for index = count - 1 to 1 by -1 do
    (
        local speed = speeds[index + 1]
        local brake = vrigLongitudinalAvailable (gripUsage * profile.braking) speed curvatures[index + 1] lateralLimit
        local span = distances[index + 1] - distances[index]
        speeds[index] = amin speeds[index] (sqrt (speed * speed + 2.0 * brake * span))
    )
    speeds
)

fn vrigTravelTimes distances speeds =
(
    local times = #(0.0)
    for index = 1 to distances.count - 1 do
    (
        local span = distances[index + 1] - distances[index]
        local mean = (speeds[index] + speeds[index + 1]) / 2.0
        local step = if mean <= 1e-6 then 0.0 else (span / mean)
        append times (times[index] + step)
    )
    times
)

-- Heading and slope come from the pose the vehicle actually holds (both
-- axles on the path), so speed, steering and body load follow what the car
-- does at a corner knot or a crest rather than an instantaneous kink.
fn vrigPlanMotion path profile targetSpeed gripUsage =
(
    local count = path.points.count
    local yaws = #()
    local slopes = #()
    local previous = undefined
    for index = 1 to count do
    (
        local pose = vrigAxlePose path path.distances[index] profile.wheelbase
        local yaw = pose[2]
        if previous != undefined do yaw = vrigUnwrapDegrees yaw previous
        append yaws yaw
        append slopes pose[3]
        previous = yaw
    )
    local curvatures = #()
    local verticalCurvatures = #()
    for index = 1 to count do
    (
        local low = amax 1 (index - 1)
        local high = amin count (index + 1)
        local span = path.distances[high] - path.distances[low]
        append curvatures (if span <= 1e-9 then 0.0 else ((degToRad (yaws[high] - yaws[low])) / span))
        append verticalCurvatures (if span <= 1e-9 then 0.0 else ((degToRad (slopes[high] - slopes[low])) / span))
    )
    local speeds = vrigSpeedProfile path.distances curvatures profile targetSpeed gripUsage
    local times = vrigTravelTimes path.distances speeds
    VehicleRigMotion path:path yaws:yaws curvatures:curvatures slopes:slopes \
        verticalCurvatures:verticalCurvatures speeds:speeds times:times duration:times[times.count]
)

-- Distance, speed, curvature and vertical curvature `elapsed` seconds into
-- the drive; past the end the vehicle stays parked at the last point.
fn vrigStateAtTime motion elapsed =
(
    local times = motion.times
    local distances = motion.path.distances
    local count = times.count
    if elapsed <= 0.0 then
        #(distances[1], motion.speeds[1], motion.curvatures[1], motion.verticalCurvatures[1])
    else if elapsed >= times[count] then
        #(distances[count], 0.0, motion.curvatures[count], motion.verticalCurvatures[count])
    else
    (
        local low = 1
        local high = count
        while (high - low) > 1 do
        (
            local middle = (low + high) / 2
            if times[middle] <= elapsed then low = middle else high = middle
        )
        local span = times[high] - times[low]
        local blend = if span <= 1e-12 then 0.0 else ((elapsed - times[low]) / span)
        #(distances[low] + blend * (distances[high] - distances[low]), \
          motion.speeds[low] + blend * (motion.speeds[high] - motion.speeds[low]), \
          motion.curvatures[low] + blend * (motion.curvatures[high] - motion.curvatures[low]), \
          motion.verticalCurvatures[low] + blend * (motion.verticalCurvatures[high] - motion.verticalCurvatures[low]))
    )
)

-- Spring-damper step mirroring vehicle_dynamics.suspension_step. The body
-- eases toward the bounded target pose instead of snapping to it.
-- `state` is #(value, velocity).
fn vrigSuspensionStep state target dt naturalHz dampingRatio =
(
    local omega = 2.0 * pi * naturalHz
    local steps = amax 1 ((ceil (dt * omega * 4.0)) as integer)
    local stepSeconds = dt / steps
    local value = state[1]
    local velocity = state[2]
    for index = 1 to steps do
    (
        local acceleration = omega * omega * (target - value) - 2.0 * dampingRatio * omega * velocity
        velocity += acceleration * stepSeconds
        value += velocity * stepSeconds
    )
    #(value, velocity)
)

-- How far one corner rises for a body pose; pitch is nose-up and roll
-- lifts the left side, both in degrees. `corner` is #(forward, left) in
-- metres from the body pivot. Mirrors vehicle_dynamics._corner_rise.
fn vrigCornerRise heave pitch roll corner =
(
    heave + corner[1] * (tan pitch) + corner[2] * (tan roll)
)

-- The body heights that keep every corner within its travel for a lean
-- scaled by `share`, as #(lowest, highest).
fn vrigHeightWindow pitch roll share corners compressionLimits extensionLimit =
(
    local lowest = -1e30
    local highest = 1e30
    for index = 1 to corners.count do
    (
        local tilt = vrigCornerRise 0.0 (pitch * share) (roll * share) corners[index]
        lowest = amax lowest (-compressionLimits[index] - tilt)
        highest = amin highest (extensionLimit - tilt)
    )
    #(lowest, highest)
)

-- Keeps each corner within its travel, lifting or lowering the body before
-- reducing the lean, so a wheel arch never comes down onto its tyre.
-- Mirrors vehicle_dynamics.fit_body_pose; returns #(heave, pitch, roll).
fn vrigFitBodyPose heave pitch roll corners compressionLimits extensionLimit =
(
    local share = 1.0
    local window = vrigHeightWindow pitch roll 1.0 corners compressionLimits extensionLimit
    if window[1] > window[2] do
    (
        local low = 0.0
        local high = 1.0
        for iteration = 1 to 40 do
        (
            local middle = (low + high) / 2.0
            local trial = vrigHeightWindow pitch roll middle corners compressionLimits extensionLimit
            if trial[1] <= trial[2] then low = middle else high = middle
        )
        share = low
        window = vrigHeightWindow pitch roll share corners compressionLimits extensionLimit
    )
    #(amin (amax heave window[1]) window[2], pitch * share, roll * share)
)

/* ------------------------------------------------------------------ *
 * Path preflight. Knot errors are reported by knot number; curvature
 * warnings are merged into distance ranges along the path.
 * ------------------------------------------------------------------ */

-- Merges flagged sample indices into #(firstIndex, lastIndex) runs.
fn vrigFlagRuns flags =
(
    local runs = #()
    local start = 0
    for index = 1 to flags.count do
    (
        if flags[index] and start == 0 do start = index
        if (not flags[index]) and start > 0 do
        (
            append runs #(start, index - 1)
            start = 0
        )
    )
    if start > 0 do append runs #(start, flags.count)
    runs
)

fn vrigMetres value = ( formattedPrint value format:".1f" ) + " m"

fn vrigNearestKnot knotPoints location =
(
    local best = 1
    local bestDistance = 1e30
    for index = 1 to knotPoints.count do
    (
        local d = distance knotPoints[index] location
        if d < bestDistance do ( bestDistance = d; best = index )
    )
    best
)

-- Returns #(issues, motion) where motion is undefined when the path has
-- errors. `vehicleLength` is in metres and may be undefined. `steerLimits`
-- is #(left, right): the most centre steering the model's wheel arches
-- allow each way, or undefined before the clearance is known.
fn vrigPreflightPath splineNode profile scale sampleCount targetSpeed vehicleLength gripUsage steerLimits =
(
    local issues = #()
    local knotPoints = vrigKnotPoints splineNode scale
    for knotPoint in knotPoints do
        if not ((vrigIsFinite knotPoint.x) and (vrigIsFinite knotPoint.y) and (vrigIsFinite knotPoint.z)) do
        (
            append issues (VehicleRigIssue code:"invalid_coordinate" severity:"error" \
                message:"Path coordinates must be finite" index:undefined)
            return #(issues, undefined)
        )

    -- Duplicate and zero-length knots are errors on the knots themselves.
    for issue in (vrigPreflightOpenPath knotPoints profile) where issue.severity == "error" do
        append issues issue
    if (vrigCountBySeverity issues "error") > 0 do return #(issues, undefined)

    local count = vrigSampleCountFor splineNode sampleCount scale
    local path = vrigMakePath (vrigSamplePath splineNode count scale)
    local motion = vrigPlanMotion path profile targetSpeed gripUsage

    if vehicleLength != undefined and path.length < vehicleLength do
        append issues (VehicleRigIssue code:"path_shorter_than_vehicle" severity:"warning" \
            message:("The path is " + (vrigMetres path.length) + " long but the vehicle is " + \
                (vrigMetres vehicleLength) + "; it will barely move") index:undefined)

    -- Steering: the lock this vehicle needs to hold the path at the planned
    -- speed, against the tighter of its preset lock and its wheel arches.
    local leftLimit = profile.maxSteer
    local rightLimit = profile.maxSteer
    if steerLimits != undefined do
    (
        leftLimit = amin leftLimit steerLimits[1]
        rightLimit = amin rightLimit steerLimits[2]
    )
    local needs = #()
    local flags = #()
    for index = 1 to motion.curvatures.count do
    (
        local curvature = motion.curvatures[index]
        local speed = motion.speeds[index]
        local needed = (atan (profile.wheelbase * (abs curvature))) + \
            profile.understeer * speed * speed * (abs curvature) / (vrigGravity())
        append needs needed
        append flags (needed > (if curvature >= 0.0 then leftLimit else rightLimit))
    )
    for run in (vrigFlagRuns flags) do
    (
        local worst = 0.0
        for index = run[1] to run[2] do worst = amax worst needs[index]
        local turningLeft = motion.curvatures[run[1]] >= 0.0
        local limit = if turningLeft then leftLimit else rightLimit
        local cause = if limit < profile.maxSteer then "the model's wheel arches allow" else "this vehicle has"
        append issues (VehicleRigIssue code:"steering_limit" severity:"warning" \
            message:("From " + (vrigMetres path.distances[run[1]]) + " to " + (vrigMetres path.distances[run[2]]) + \
                " the path needs " + (formattedPrint worst format:".0f") + " deg of steering; " + cause + " " + \
                (formattedPrint limit format:".0f") + " deg, so the wheels will stop there") \
            index:(vrigNearestKnot knotPoints path.points[run[1]]))
    )

    -- Corners: the spline turns more than 45 degrees within one metre.
    for knot = 2 to knotPoints.count - 1 do
    (
        local nearest = 1
        local nearestDistance = 1e30
        for index = 1 to path.points.count do
        (
            local d = distance path.points[index] knotPoints[knot]
            if d < nearestDistance do ( nearestDistance = d; nearest = index )
        )
        local here = path.distances[nearest]
        local incoming = (vrigPathPoint path here) - (vrigPathPoint path (here - 0.5))
        local outgoing = (vrigPathPoint path (here + 0.5)) - (vrigPathPoint path here)
        if (length incoming) > 1e-6 and (length outgoing) > 1e-6 do
        (
            local turn = acos (amax -1.0 (amin 1.0 (dot (normalize incoming) (normalize outgoing))))
            if turn >= 45.0 do
                append issues (VehicleRigIssue code:"sharp_kink" severity:"warning" \
                    message:("The path turns " + (formattedPrint turn format:".0f") + \
                        " deg at this knot; the vehicle will round it over its wheelbase") index:knot)
        )
    )

    -- Crossings, on a decimated copy so long paths stay quick.
    local stride = amax 1 (path.points.count / 300)
    local coarse = for index = 1 to path.points.count by stride collect path.points[index]
    if (findItem coarse path.points[path.points.count]) == 0 do append coarse path.points[path.points.count]
    local crossed = false
    for first = 1 to coarse.count - 3 while not crossed do
        for second = first + 2 to coarse.count - 1 while not crossed do
            if vrigSegmentsIntersect2D coarse[first] coarse[first + 1] coarse[second] coarse[second + 1] do
            (
                crossed = true
                append issues (VehicleRigIssue code:"self_intersection" severity:"warning" \
                    message:"The path crosses itself in the travel plane" \
                    index:(vrigNearestKnot knotPoints coarse[second]))
            )

    #(issues, motion)
)

/* ------------------------------------------------------------------ *
 * Restore records. User properties keep the data with the scene, so a
 * saved and reopened rig can still be removed. A record is written once:
 * re-baking must never replace the original with a rigged pose.
 * ------------------------------------------------------------------ */

fn vrigMatrixToString transform =
(
    local parts = #()
    for row in #(transform.row1, transform.row2, transform.row3, transform.row4) do
    (
        append parts (row.x as string)
        append parts (row.y as string)
        append parts (row.z as string)
    )
    local text = parts[1]
    for index = 2 to parts.count do text += "," + parts[index]
    text
)

fn vrigStringToMatrix text =
(
    local parts = filterString text ","
    if parts.count != 12 then undefined
    else
    (
        local values = for part in parts collect (part as float)
        matrix3 [values[1], values[2], values[3]] [values[4], values[5], values[6]] \
            [values[7], values[8], values[9]] [values[10], values[11], values[12]]
    )
)

fn vrigIsBakeHelper node =
(
    (findItem #("VRIG_Master", "VRIG_Body", "VRIG_Steering", "VRIG_PathGuide", \
        "VRIG_SteeringColumn", "VRIG_SteeringSpin") node.name) > 0 or \
        (matchPattern node.name pattern:"VRIG_WheelPivot_*") or \
        (matchPattern node.name pattern:"VRIG_WheelSpin_*")
)

-- Assembly and bake keep separate records, so a node bound by both returns
-- to its pre-bake pose on unbake and to its imported pose on removal.
fn vrigRecordKeys stage =
(
    if stage == "bake" then
        #("VRIG_baked", "VRIG_bakeParent", "VRIG_bakeParentHandle", "VRIG_bakeTransform")
    else
        #("VRIG_bound", "VRIG_originalParent", "VRIG_originalParentHandle", "VRIG_originalTransform")
)

-- `stage` is "assembly" or "bake". Returns false when a record exists.
fn vrigRecordOriginalState node stage =
(
    local keys = vrigRecordKeys stage
    if (getUserProp node keys[1]) != undefined then false
    else
    (
        local originalParent = node.parent
        local originalTransform = vrigMatrixToString node.transform
        setUserProp node keys[2] (if originalParent == undefined then "" else originalParent.name)
        setUserProp node keys[3] (if originalParent == undefined then 0 else originalParent.inode.handle)
        setUserProp node keys[4] originalTransform
        setUserProp node keys[1] stage
        true
    )
)

fn vrigRecordedParent node keys =
(
    local parentNode = undefined
    local handle = getUserProp node keys[3]
    if handle != undefined and handle != 0 do parentNode = maxOps.getNodeByHandle handle
    -- Fall back to the name only when the handle no longer resolves; names
    -- repeat freely in imported scenes.
    if parentNode == undefined or (isDeleted parentNode) do
    (
        local parentName = getUserProp node keys[2]
        parentNode = if parentName == undefined or (parentName as string) == "" then undefined \
            else getNodeByName (parentName as string)
    )
    parentNode
)

-- Records written before stages existed are bake bindings while the node
-- still hangs from a bake helper.
fn vrigIsLegacyBakeRecord node =
(
    local tag = getUserProp node "VRIG_bound"
    tag != undefined and (tag as string) == "true" and node.parent != undefined and (vrigIsBakeHelper node.parent)
)

-- Restores one stage's records. Nodes are unlinked before any transform is
-- restored, so restoring a parent can never drag an already-restored child.
fn vrigRestoreOriginalStates stage =
(
    local keys = vrigRecordKeys stage
    local records = #()
    for node in objects do
    (
        if (getUserProp node keys[1]) != undefined then append records node
        else if stage == "bake" and (vrigIsLegacyBakeRecord node) do append records node
    )
    local recordKeys = for node in records collect \
        (if (getUserProp node keys[1]) != undefined then keys else (vrigRecordKeys "assembly"))
    local parents = for index = 1 to records.count collect (vrigRecordedParent records[index] recordKeys[index])
    for node in records do node.parent = undefined
    for index = 1 to records.count do
    (
        local text = getUserProp records[index] recordKeys[index][4]
        local restored = if text == undefined then undefined else vrigStringToMatrix (text as string)
        if restored != undefined do records[index].transform = restored
    )
    for index = 1 to records.count do
    (
        local node = records[index]
        node.parent = parents[index]
        for key in recordKeys[index] do deleteUserProp node key
    )
    records.count
)

fn vrigBakeHelpers = for node in objects where (vrigIsBakeHelper node) collect node

-- Removes the baked layer only: bound nodes return to their pre-bake
-- parents and the bake helpers are deleted. An assembled import stays
-- assembled, so a second bake starts from the same state as the first.
fn vrigUnbake =
(
    local restoredCount = vrigRestoreOriginalStates "bake"
    local helpers = vrigBakeHelpers()
    local deletedCount = helpers.count
    if deletedCount > 0 do delete helpers
    #(restoredCount, deletedCount)
)

fn vrigGeneratedNodes =
(
    for node in objects where ((matchPattern node.name pattern:"VRIG_*") and not (matchPattern node.name pattern:"VRIG_Test_*")) collect node
)

fn vrigGeneratedGroupHeads =
(
    for node in objects where ((isGroupHead node) and (matchPattern node.name pattern:"VRIG_*") and not (matchPattern node.name pattern:"VRIG_Test_*")) collect node
)

fn vrigRemoveGeneratedRig =
(
    local unbaked = vrigUnbake()
    -- Deleting a group head directly can leave imported meshes marked as
    -- group members. Ungroup first, then restore the recorded artist state.
    for groupHead in (vrigGeneratedGroupHeads()) do ungroup groupHead
    local restoredCount = unbaked[1] + (vrigRestoreOriginalStates "assembly")

    local generated = vrigGeneratedNodes()
    local generatedCount = unbaked[2] + generated.count
    if generated.count > 0 do delete generated

    #(restoredCount, generatedCount)
)

/* ------------------------------------------------------------------ *
 * Vehicle measurement
 * ------------------------------------------------------------------ */

fn vrigIsDescendantOf node root =
(
    local current = node
    local found = false
    while current != undefined and not found do
    (
        if current == root do found = true
        current = current.parent
    )
    found
)

-- Imported vehicles routinely have arbitrary object pivots. Measure the
-- evaluated mesh bounds instead of trusting any pivot.
fn vrigGeometryUnder root =
(
    for node in geometry where (node == root or vrigIsDescendantOf node root) collect node
)

-- Identity coordinates request evaluated world bounds.
fn vrigWorldBounds nodes =
(
    local low = [1e30, 1e30, 1e30]
    local high = [-1e30, -1e30, -1e30]
    for node in nodes do
    (
        local bounds = nodeGetBoundingBox node (matrix3 1)
        low = [amin low.x bounds[1].x, amin low.y bounds[1].y, amin low.z bounds[1].z]
        high = [amax high.x bounds[2].x, amax high.y bounds[2].y, amax high.z bounds[2].z]
    )
    #(low, high)
)

fn vrigBoundsCenter nodes =
(
    local bounds = vrigWorldBounds nodes
    (bounds[1] + bounds[2]) / 2.0
)

-- Model axes as #(component, sign), component 1 = X and 2 = Y. The left
-- axis is up x forward; positive rotation about it rolls a wheel forward.
fn vrigForwardAxis forwardIndex =
(
    case forwardIndex of ( 1: #(1, 1.0); 2: #(1, -1.0); 3: #(2, 1.0); 4: #(2, -1.0); default: #(1, 1.0) )
)

fn vrigLeftAxis forwardIndex =
(
    case forwardIndex of ( 1: #(2, 1.0); 2: #(2, -1.0); 3: #(1, -1.0); 4: #(1, 1.0); default: #(2, 1.0) )
)

fn vrigAxisVector axis =
(
    local vector = [0, 0, 0]
    vector[axis[1]] = axis[2]
    vector
)

-- Yaw that turns the modelled forward axis onto world +X.
fn vrigForwardOffsetDegrees forwardIndex =
(
    case forwardIndex of ( 1: 0.0; 2: 180.0; 3: -90.0; 4: 90.0; default: 0.0 )
)

-- Wheel labels give the facing: FL/FR averaged against RL/RR. Returns the
-- dialog index for +X, -X, +Y or -Y.
fn vrigForwardIndexFromCentres wheelCentres =
(
    local forward = ((wheelCentres[1] + wheelCentres[2]) / 2.0) - ((wheelCentres[3] + wheelCentres[4]) / 2.0)
    if (abs forward.x) >= (abs forward.y) then
        (if forward.x >= 0.0 then 1 else 2)
    else
        (if forward.y >= 0.0 then 3 else 4)
)

-- Everything below is in scene units. Returns #(ok, message, measure).
fn vrigMeasureVehicle bodyNode wheelNodes =
(
    local wheelTokens = #("FL", "FR", "RL", "RR")
    local wheelCentres = #()
    local radii = #()
    local floors = #()
    local wheelGeometry = #()
    for index = 1 to 4 do
    (
        if wheelNodes[index] == undefined or not (isValidNode wheelNodes[index]) do
            return #(false, ("Wheel " + wheelTokens[index] + " is not assigned"), undefined)
        local nodes = vrigGeometryUnder wheelNodes[index]
        if nodes.count == 0 do
            return #(false, ("Wheel " + wheelTokens[index] + " contains no geometry"), undefined)
        join wheelGeometry nodes
        local bounds = vrigWorldBounds nodes
        append wheelCentres ((bounds[1] + bounds[2]) / 2.0)
        append radii ((bounds[2].z - bounds[1].z) / 2.0)
        append floors bounds[1].z
    )

    local bodyGeometry = for node in (vrigGeometryUnder bodyNode) where (findItem wheelGeometry node) == 0 collect node
    if bodyGeometry.count == 0 do return #(false, "The body contains no geometry apart from the wheels", undefined)

    local tyreRadius = (radii[1] + radii[2] + radii[3] + radii[4]) / 4.0
    if tyreRadius <= 0.0 do return #(false, "The wheels have no measurable radius", undefined)
    local groundZ = amin (amin floors[1] floors[2]) (amin floors[3] floors[4])
    local frontMid = (wheelCentres[1] + wheelCentres[2]) / 2.0
    local rearMid = (wheelCentres[3] + wheelCentres[4]) / 2.0
    local axis = frontMid - rearMid
    local wheelbase = sqrt (axis.x * axis.x + axis.y * axis.y)
    if wheelbase <= tyreRadius do
        return #(false, "The front and rear wheels are not separated; check the FL/FR/RL/RR assignment", undefined)
    local frontSpan = wheelCentres[1] - wheelCentres[2]
    local rearSpan = wheelCentres[3] - wheelCentres[4]
    local track = ((sqrt (frontSpan.x * frontSpan.x + frontSpan.y * frontSpan.y)) + \
        (sqrt (rearSpan.x * rearSpan.x + rearSpan.y * rearSpan.y))) / 2.0
    local forwardIndex = vrigForwardIndexFromCentres wheelCentres
    local centre = (frontMid + rearMid) / 2.0
    local bodyBounds = vrigWorldBounds bodyGeometry
    local allBounds = vrigWorldBounds (join (copy bodyGeometry #nomap) wheelGeometry)
    local forwardVector = vrigAxisVector (vrigForwardAxis forwardIndex)
    local vehicleLength = abs (dot (allBounds[2] - allBounds[1]) forwardVector)

    #(true, "", VehicleRigMeasure wheelCentres:wheelCentres tyreRadius:tyreRadius \
        groundZ:groundZ groundCentre:[centre.x, centre.y, groundZ] \
        wheelbase:wheelbase track:track forwardIndex:forwardIndex \
        bodyCentre:((bodyBounds[1] + bodyBounds[2]) / 2.0) \
        axleZ:((wheelCentres[1].z + wheelCentres[2].z + wheelCentres[3].z + wheelCentres[4].z) / 4.0) \
        length:vehicleLength)
)

-- A model built at a different scale from the scene's unit setting gives
-- an implausible wheelbase. Fit the scale to the preset wheelbase in that
-- case, otherwise trust the scene. Returns #(scale, wasFitted).
fn vrigSuggestedScale measure declaredScale presetWheelbase =
(
    local wheelbaseMetres = measure.wheelbase / declaredScale
    if wheelbaseMetres >= 1.5 and wheelbaseMetres <= 10.0 then #(declaredScale, false)
    else #(measure.wheelbase / presetWheelbase, true)
)

-- Preset dynamics with the measured geometry, so steering matches the model.
fn vrigMeasuredProfile preset measure scale tyreRadius =
(
    local profile = vrigCopyProfile preset
    profile.wheelbase = measure.wheelbase / scale
    profile.track = measure.track / scale
    profile.tyreRadius = tyreRadius
    profile
)

fn vrigDescribeMeasure measure scale =
(
    "Vehicle: wheelbase " + (vrigMetres (measure.wheelbase / scale)) + ", track " + \
        (vrigMetres (measure.track / scale)) + ", tyre radius " + \
        (formattedPrint (measure.tyreRadius / scale) format:".3f") + " m, length " + \
        (vrigMetres (measure.length / scale)) + " at " + (formattedPrint scale format:".3f") + " units/m"
)

/* ------------------------------------------------------------------ *
 * Wheel-arch clearance. Imported models often fit their arches within a
 * few centimetres of the tyres, so the body may only move as far as the
 * model itself allows.
 * ------------------------------------------------------------------ */

-- Session cache, so repeated bakes of one vehicle skip the measurement
-- without writing anything into the scene.
global vrigClearanceCache = #()

-- Tyre envelope: a cylinder around the measured wheel, sampled on its
-- tread and both sidewalls. `axle` is the unit axle direction.
fn vrigTyreEnvelope centre radius halfWidth axle =
(
    local up = [0, 0, 1]
    local along = normalize (cross axle up)
    local points = #()
    for step = 0 to 47 do
    (
        local angle = step * 7.5
        local radial = along * (cos angle) + up * (sin angle)
        for offset in #(-0.9, 0.0, 0.9) do
            append points (centre + radial * radius + axle * (offset * halfWidth))
        if (mod step 2) == 0 do
            for side in #(-1.0, 1.0) do
                append points (centre + radial * (radius * 0.85) + axle * (side * halfWidth))
    )
    points
)

-- One ray grid per mesh: a grid keeps only the first node added to it.
fn vrigMeshGrids nodes =
(
    for node in nodes collect
    (
        local grid = RayMeshGridIntersect()
        grid.initialize 24
        grid.addNode node
        grid.buildGrid()
        grid
    )
)

fn vrigFreeGrids grids = ( for grid in grids do grid.free() )

fn vrigNodeNear node centre reach =
(
    local bounds = nodeGetBoundingBox node (matrix3 1)
    bounds[1].x <= centre.x + reach and bounds[2].x >= centre.x - reach and \
    bounds[1].y <= centre.y + reach and bounds[2].y >= centre.y - reach and \
    bounds[1].z <= centre.z + reach and bounds[2].z >= centre.z - reach
)

-- True when the tyre, turned by `angle` about the vertical through its
-- centre and raised by `lift` into the arch, crosses any grid.
fn vrigTyreCollides grids points centre angle lift =
(
    local rotation = rotateZMatrix angle
    local origin = centre + [0, 0, lift]
    local hit = false
    for tyrePoint in points while not hit do
    (
        local moved = ((tyrePoint - centre) * rotation) + origin
        for grid in grids while not hit do
            if (grid.intersectSegment origin moved true) > 0 do hit = true
    )
    hit
)

-- Largest steering angle in `direction` (+1 left, -1 right) before the
-- tyre, already raised by `lift`, touches the body.
fn vrigSteerClearance grids points centre direction lift =
(
    if vrigTyreCollides grids points centre 0.0 lift then 0.0
    else if not (vrigTyreCollides grids points centre (direction * 60.0) lift) then 60.0
    else
    (
        local low = 0.0
        local high = 60.0
        for iteration = 1 to 12 do
        (
            local middle = (low + high) / 2.0
            if vrigTyreCollides grids points centre (direction * middle) lift then high = middle else low = middle
        )
        low
    )
)

-- Measures how far each corner may compress and how far each front wheel
-- may steer before the model's own arches meet the tyre. A mesh that
-- already crosses a tyre at rest, such as a liner modelled inside the
-- wheel, is left out for that wheel: the modeller accepted it, and
-- counting it would forbid any motion at all.
fn vrigMeasureClearance bodyNodes wheelNodes measure profile scale =
(
    local started = timeStamp()
    local leftVector = vrigAxisVector (vrigLeftAxis measure.forwardIndex)
    local radius = measure.tyreRadius
    local reach = radius * 2.2
    local candidates = #()
    for node in bodyNodes do
    (
        local close = false
        for centre in measure.wheelCentres while not close do close = vrigNodeNear node centre reach
        if close do append candidates node
    )
    local grids = vrigMeshGrids candidates
    local travelLimit = profile.springTravel * scale
    local compression = #()
    local gaps = #()
    local clear = #()
    local ignored = #()
    for index = 1 to 4 do
    (
        local centre = measure.wheelCentres[index]
        local bounds = vrigWorldBounds (vrigGeometryUnder wheelNodes[index])
        local halfWidth = (abs (dot (bounds[2] - bounds[1]) leftVector)) / 2.0
        local points = vrigTyreEnvelope centre radius halfWidth leftVector
        local active = #()
        for meshIndex = 1 to candidates.count where (vrigNodeNear candidates[meshIndex] centre reach) do
        (
            local crossing = false
            for tyrePoint in points while not crossing do
                if (grids[meshIndex].intersectSegment centre tyrePoint true) > 0 do crossing = true
            if crossing then appendIfUnique ignored candidates[meshIndex].name
            else append active grids[meshIndex]
        )
        local gap = 1e30
        for tyrePoint in points where tyrePoint.z > centre.z + radius * 0.2 do
            for grid in active do
                if (grid.intersectRay tyrePoint [0, 0, 1] true) > 0 do
                    gap = amin gap (grid.getHitDist (grid.getClosestHit()))
        append gaps gap
        -- Keep a margin for the space between envelope samples.
        local allowed = if gap >= 1e29 then travelLimit else (amax 0.0 (gap - (amax (0.005 * scale) (gap * 0.15))))
        append compression (amin allowed travelLimit)
        if index <= 2 then
        (
            local leftClear = amax 0.0 ((vrigSteerClearance active points centre 1.0 compression[index]) - 1.5)
            local rightClear = amax 0.0 ((vrigSteerClearance active points centre -1.0 compression[index]) - 1.5)
            append clear #(leftClear, rightClear)
        )
        else append clear undefined
    )
    vrigFreeGrids grids
    -- A left turn puts the left wheel on the inside.
    VehicleRigClearance compression:compression archGaps:gaps \
        leftTurnLimits:#(clear[1][1], clear[2][1]) rightTurnLimits:#(clear[2][2], clear[1][2]) \
        ignored:ignored seconds:((timeStamp() - started) / 1000.0)
)

fn vrigClearanceKey rootNode measure profile scale bodyNodes =
(
    local key = (rootNode.inode.handle as string) + "|" + profile.name + "|" + (scale as string) + "|" + \
        (bodyNodes.count as string) + "|" + (measure.tyreRadius as string)
    for centre in measure.wheelCentres do key += "|" + ((centre - measure.groundCentre) as string)
    key
)

-- The clearance for this vehicle, measured once per layout per session.
fn vrigCachedClearance rootNode bodyNodes wheelNodes measure profile scale =
(
    local key = vrigClearanceKey rootNode measure profile scale bodyNodes
    local found = undefined
    for entry in vrigClearanceCache where entry[1] == key do found = entry[2]
    if found == undefined do
    (
        found = vrigMeasureClearance bodyNodes wheelNodes measure profile scale
        append vrigClearanceCache #(key, found)
    )
    found
)

-- Centre steering the arches allow, as #(left, right) degrees.
fn vrigSteerLimits clearance profile =
(
    #(vrigMaxCentreSteer profile clearance.leftTurnLimits[1] clearance.leftTurnLimits[2], \
      vrigMaxCentreSteer profile clearance.rightTurnLimits[1] clearance.rightTurnLimits[2])
)

fn vrigGapText gap scale = if gap >= 1e29 then "open" else ((formattedPrint (gap / scale * 100.0) format:".1f") + " cm")

fn vrigDescribeClearance clearance profile scale =
(
    local limits = vrigSteerLimits clearance profile
    local text = "Wheel arches: " + (vrigGapText (amin clearance.archGaps[1] clearance.archGaps[2]) scale) + \
        " above the front tyres, " + (vrigGapText (amin clearance.archGaps[3] clearance.archGaps[4]) scale) + \
        " above the rear; steering lock " + (formattedPrint limits[1] format:".0f") + " deg left, " + \
        (formattedPrint limits[2] format:".0f") + " deg right"
    if clearance.ignored.count > 0 do
    (
        text += ". Already inside the tyres at rest, so not counted: "
        for index = 1 to clearance.ignored.count do
        (
            local parts = filterString clearance.ignored[index] ":"
            text += (if index > 1 then ", " else "") + parts[parts.count]
        )
    )
    text + "."
)

-- Body meshes for clearance: everything under the body that is not part
-- of a wheel, a brake or the steering wheel.
fn vrigClearanceBodyNodes bodyNode wheelNodes extraNodes =
(
    local excluded = #()
    for node in (join (copy wheelNodes #nomap) extraNodes) where node != undefined do
        join excluded (vrigGeometryUnder node)
    for node in (vrigGeometryUnder bodyNode) where (findItem excluded node) == 0 collect node
)

/* ------------------------------------------------------------------ *
 * Steering wheel
 * ------------------------------------------------------------------ */

fn vrigCovarianceTimes rows vector = [dot rows[1] vector, dot rows[2] vector, dot rows[3] vector]

fn vrigDominantAxis rows seed =
(
    local axis = normalize seed
    local settled = false
    for iteration = 1 to 80 while not settled do
    (
        local product = vrigCovarianceTimes rows axis
        if (length product) < 1e-12 then settled = true
        else
        (
            local updated = normalize product
            if (distance updated axis) < 1e-10 do settled = true
            axis = updated
        )
    )
    axis
)

-- Normal of the plane a set of points lies closest to: the axis a
-- steering wheel turns about. Power iteration finds the two directions the
-- rim spreads along; their cross product is the column.
fn vrigPlaneNormal points =
(
    local mean = [0, 0, 0]
    for p in points do mean += p
    mean = mean / points.count
    local rows = #([0, 0, 0], [0, 0, 0], [0, 0, 0])
    for p in points do
    (
        local d = p - mean
        rows[1] += d * d.x
        rows[2] += d * d.y
        rows[3] += d * d.z
    )
    local first = vrigDominantAxis rows [1.0, 0.31, 0.17]
    local firstValue = dot first (vrigCovarianceTimes rows first)
    local deflated = for index = 1 to 3 collect (rows[index] - first * (firstValue * first[index]))
    local second = vrigDominantAxis deflated (cross first [0.29, 0.13, 1.0])
    normalize (cross first second)
)

-- Centre and turning axis of a steering wheel, the axis pointing back
-- toward the driver. Returns #(centre, axis), or undefined without geometry.
fn vrigSteeringAxis steeringNode forwardVector =
(
    local nodes = vrigGeometryUnder steeringNode
    if nodes.count == 0 then undefined
    else
    (
        local points = #()
        for node in nodes do
        (
            local snapshot = snapshotAsMesh node
            local stride = amax 1 (snapshot.numverts / 3000)
            for index = 1 to snapshot.numverts by stride do append points (getVert snapshot index)
        )
        local axis = vrigPlaneNormal points
        if (dot axis forwardVector) > 0.0 do axis = -axis
        #(vrigBoundsCenter nodes, axis)
    )
)

-- A frame whose Z axis is `axis`, at `position`.
fn vrigAxisMatrix axis position =
(
    local zAxis = normalize axis
    local reference = if (abs zAxis.z) > 0.9 then [1, 0, 0] else [0, 0, 1]
    local xAxis = normalize (cross reference zAxis)
    local yAxis = cross zAxis xAxis
    matrix3 xAxis yAxis zAxis position
)

/* ------------------------------------------------------------------ *
 * Imported vehicle assembly
 * ------------------------------------------------------------------ */

-- Imported marketplace vehicles often arrive as independent meshes.  This
-- creates only generated parents; source geometry is neither combined nor
-- renamed, and every reparented source node is restoration-recorded first.
fn vrigWheelMatchCount node =
(
    local lowered = toLower node.name
    local count = 0
    for token in #( "wheelfl", "wheelfr", "wheelbl", "wheelbr" ) do
        if (findString lowered token) != undefined do count += 1
    count
)

fn vrigClassifyWheelNode node =
(
    local lowered = toLower node.name
    if (findString lowered "wheelfl") != undefined then 1
    else if (findString lowered "wheelfr") != undefined then 2
    else if (findString lowered "wheelbl") != undefined then 3
    else if (findString lowered "wheelbr") != undefined then 4
    else 0
)

-- Calipers and brake parts steer with the wheel but must not spin with it.
fn vrigIsBrakePart node =
(
    local lowered = toLower node.name
    (findString lowered "caliper") != undefined or (findString lowered "brake") != undefined
)

fn vrigTopAssemblyNode node =
(
    local top = node
    while top.parent != undefined do top = top.parent
    top
)

fn vrigTopAssemblyNodes nodes =
(
    local tops = #()
    for node in nodes do
    (
        local top = vrigTopAssemblyNode node
        if (findItem tops top) == 0 do append tops top
    )
    tops
)

fn vrigAssemblyPoint pointName size colourValue position parentNode =
(
    local helper = point name:pointName size:size box:off cross:on axistripod:on \
        centermarker:off wirecolor:colourValue
    helper.position = position
    if parentNode != undefined do helper.parent = parentNode
    helper
)

-- A steering wheel turns with the front wheels; its parts ride on the body.
fn vrigIsSteeringWheelPart node =
(
    local compact = substituteString (substituteString (substituteString (toLower node.name) "_" "") " " "") "-" ""
    (findString compact "steeringwheel") != undefined
)

fn vrigAssemblyFailure message = #(false, message, undefined, undefined, #(), #(), undefined)

-- Returns #(ok, message, root, body, wheels, brakes, steeringWheel);
-- brakes[i] is undefined when a wheel has no caliper parts, and
-- steeringWheel is undefined when the import has none.
fn vrigAssembleSelectedVehicle vehicleName =
(
    local wheelTokens = #("FL", "FR", "RL", "RR")
    local meshNodes = for node in selection where (superClassOf node == GeometryClass) and \
        not (matchPattern node.name pattern:"VRIG_*") collect node
    if meshNodes.count == 0 do
        return (vrigAssemblyFailure "Select the imported vehicle mesh nodes before assembly.")

    local spinSets = #(#(), #(), #(), #())
    local brakeSets = #(#(), #(), #(), #())
    local bodyMeshes = #()
    local steeringMeshes = #()
    for node in meshNodes do
    (
        if (vrigWheelMatchCount node) > 1 do
            return (vrigAssemblyFailure (node.name + " matches more than one wheel cluster."))
        local wheelIndex = vrigClassifyWheelNode node
        if wheelIndex > 0 then
        (
            if vrigIsBrakePart node then append brakeSets[wheelIndex] node
            else append spinSets[wheelIndex] node
        )
        else if vrigIsSteeringWheelPart node then append steeringMeshes node
        else append bodyMeshes node
    )
    for wheelIndex = 1 to 4 do
        if spinSets[wheelIndex].count == 0 do
            return (vrigAssemblyFailure ("Missing wheel cluster " + wheelTokens[wheelIndex] + "."))
    if bodyMeshes.count == 0 do
        return (vrigAssemblyFailure "No non-wheel body meshes were found.")

    -- Group the highest imported parents. Each must belong to exactly one
    -- cluster: one imported parent holding both a wheel and body parts
    -- cannot be split without moving source geometry.
    local bodySet = vrigTopAssemblyNodes bodyMeshes
    local steeringSet = vrigTopAssemblyNodes steeringMeshes
    local spinTops = for wheelIndex = 1 to 4 collect (vrigTopAssemblyNodes spinSets[wheelIndex])
    local brakeTops = for wheelIndex = 1 to 4 collect (vrigTopAssemblyNodes brakeSets[wheelIndex])
    local owners = #()
    local ownerNames = #()
    fn claim owners ownerNames nodes label =
    (
        local clash = undefined
        for node in nodes do
        (
            local found = findItem owners node
            if found > 0 then clash = #(node, ownerNames[found], label)
            else ( append owners node; append ownerNames label )
        )
        clash
    )
    local clashes = #()
    append clashes (claim owners ownerNames bodySet "the body")
    append clashes (claim owners ownerNames steeringSet "the steering wheel")
    for wheelIndex = 1 to 4 do
    (
        append clashes (claim owners ownerNames spinTops[wheelIndex] ("wheel " + wheelTokens[wheelIndex]))
        append clashes (claim owners ownerNames brakeTops[wheelIndex] ("brake " + wheelTokens[wheelIndex]))
    )
    for clash in clashes where clash != undefined do
        return (vrigAssemblyFailure (clash[1].name + " holds parts of both " + clash[2] + " and " + \
            clash[3] + ". Detach or ungroup them before assembly."))

    local rootName = "VRIG_" + vehicleName + "_Root"
    if getNodeByName rootName != undefined do
        return (vrigAssemblyFailure (rootName + " already exists. Remove the generated rig before assembling again."))

    -- Measure from the meshes before anything moves.
    local wheelCentres = #()
    local radii = #()
    for wheelIndex = 1 to 4 do
    (
        local bounds = vrigWorldBounds spinSets[wheelIndex]
        append wheelCentres ((bounds[1] + bounds[2]) / 2.0)
        append radii ((bounds[2].z - bounds[1].z) / 2.0)
    )
    local tyreRadius = (radii[1] + radii[2] + radii[3] + radii[4]) / 4.0
    local groundZ = (vrigWorldBounds (join (join (join (copy spinSets[1] #nomap) spinSets[2]) spinSets[3]) spinSets[4]))[1].z
    local centre = (wheelCentres[1] + wheelCentres[2] + wheelCentres[3] + wheelCentres[4]) / 4.0

    local sourceNodes = #()
    join sourceNodes bodySet
    join sourceNodes steeringSet
    for wheelIndex = 1 to 4 do ( join sourceNodes spinTops[wheelIndex]; join sourceNodes brakeTops[wheelIndex] )
    for node in sourceNodes do vrigRecordOriginalState node "assembly"

    local root = vrigAssemblyPoint rootName (tyreRadius * 2.0) (color 120 220 255) \
        [centre.x, centre.y, groundZ] undefined
    local bodyCarrier = vrigAssemblyPoint ("VRIG_" + vehicleName + "_BodyCarrier") (tyreRadius * 1.5) \
        (color 160 255 160) (vrigBoundsCenter bodyMeshes) root

    -- Native groups keep imports whose geometry is offset from a shared
    -- pivot together without touching any source transform.
    local bodyGroup = group bodySet name:("VRIG_" + vehicleName + "_BodyGroup")
    if bodyGroup == undefined do
    (
        vrigRemoveGeneratedRig()
        return (vrigAssemblyFailure "Max could not create the vehicle body group.")
    )
    bodyGroup.parent = bodyCarrier

    local steeringWheel = undefined
    if steeringSet.count > 0 do
    (
        steeringWheel = vrigAssemblyPoint ("VRIG_" + vehicleName + "_SteeringWheel") (tyreRadius * 0.6) \
            (color 255 120 200) (vrigBoundsCenter steeringMeshes) bodyCarrier
        local steeringGroup = group steeringSet name:("VRIG_" + vehicleName + "_SteeringGroup")
        if steeringGroup == undefined do
        (
            vrigRemoveGeneratedRig()
            return (vrigAssemblyFailure "Max could not create the steering wheel group.")
        )
        steeringGroup.parent = steeringWheel
    )

    local wheels = #()
    local brakes = #()
    for wheelIndex = 1 to 4 do
    (
        local token = wheelTokens[wheelIndex]
        local wheel = vrigAssemblyPoint ("VRIG_" + vehicleName + "_Wheel_" + token) (tyreRadius * 1.3) \
            (color 255 200 80) wheelCentres[wheelIndex] root
        local wheelGroup = group spinTops[wheelIndex] name:("VRIG_" + vehicleName + "_WheelGroup_" + token)
        if wheelGroup == undefined do
        (
            vrigRemoveGeneratedRig()
            return (vrigAssemblyFailure ("Max could not create wheel group " + token + "."))
        )
        wheelGroup.parent = wheel
        append wheels wheel

        local brake = undefined
        if brakeTops[wheelIndex].count > 0 do
        (
            brake = vrigAssemblyPoint ("VRIG_" + vehicleName + "_Brake_" + token) (tyreRadius * 0.9) \
                (color 255 120 80) wheelCentres[wheelIndex] root
            local brakeGroup = group brakeTops[wheelIndex] name:("VRIG_" + vehicleName + "_BrakeGroup_" + token)
            if brakeGroup == undefined do
            (
                vrigRemoveGeneratedRig()
                return (vrigAssemblyFailure ("Max could not create brake group " + token + "."))
            )
            brakeGroup.parent = brake
            setUserProp wheel "VRIG_brakeHandle" brake.inode.handle
        )
        append brakes brake
    )
    select root
    local brakeCount = 0
    for brake in brakes where brake != undefined do brakeCount += 1
    local message = "Assembled " + (sourceNodes.count as string) + " imported nodes non-destructively; " + \
        (brakeCount as string) + " wheel(s) have separate brake parts that steer without spinning"
    message += if steeringWheel != undefined then ", and the steering wheel turns with the front wheels." else "."
    #(true, message, root, bodyCarrier, wheels, brakes, steeringWheel)
)

/* ------------------------------------------------------------------ *
 * Test path fixtures
 * ------------------------------------------------------------------ */

fn vrigBuildOpenSpline splineName knotPoints isSmooth =
(
    local path = SplineShape name:splineName
    addNewSpline path
    for knotPoint in knotPoints do
        if isSmooth then addKnot path 1 #smooth #curve knotPoint else addKnot path 1 #corner #line knotPoint
    updateShape path
    path.wirecolor = color 255 160 40
    path
)

-- Knots for a road made of straights, circular arcs and smooth ramps, in
-- metres in the vehicle's frame (forward, left, up). Segments are
-- #(#straight, length), #(#arc, radius, degrees) turning left for positive
-- degrees, or #(#ramp, length, rise). Arcs use the cubic handle length
-- 4/3 tan(sweep/4) r, so a bend has a constant radius rather than the
-- uneven one auto-smoothed knots give. Returns #(point, in, out) triples.
fn vrigRoadKnots segments =
(
    local position = [0, 0, 0]
    local heading = 0.0
    local knots = #(#(position, position, position))
    for segment in segments do
    (
        local direction = [cos heading, sin heading, 0]
        case segment[1] of
        (
            #straight:
            (
                local finish = position + direction * segment[2]
                knots[knots.count][3] = position + direction * (segment[2] / 3.0)
                append knots #(finish, finish - direction * (segment[2] / 3.0), finish)
                position = finish
            )
            #arc:
            (
                local radius = segment[2]
                local pieces = (ceil ((abs segment[3]) / 90.0)) as integer
                local sweep = segment[3] / pieces
                local handle = 4.0 / 3.0 * (tan ((abs sweep) / 4.0)) * radius
                for piece = 1 to pieces do
                (
                    local startDirection = [cos heading, sin heading, 0]
                    local side = if sweep > 0.0 then 1.0 else -1.0
                    local centre = position + [-(sin heading), cos heading, 0] * (radius * side)
                    local finish = centre + ((position - centre) * (rotateZMatrix sweep))
                    heading += sweep
                    local endDirection = [cos heading, sin heading, 0]
                    knots[knots.count][3] = position + startDirection * handle
                    append knots #(finish, finish - endDirection * handle, finish)
                    position = finish
                )
            )
            #ramp:
            (
                -- Level handles at both ends: a sag at the foot, a crest at the top.
                local finish = position + direction * segment[2] + [0, 0, segment[3]]
                knots[knots.count][3] = position + direction * (segment[2] / 2.0)
                append knots #(finish, finish - direction * (segment[2] / 2.0), finish)
                position = finish
            )
        )
    )
    knots
)

fn vrigBuildRoad splineName segments origin forward left scale =
(
    fn toWorld roadPoint origin forward left scale =
    (
        origin + (forward * roadPoint.x + left * roadPoint.y + [0, 0, roadPoint.z]) * scale
    )
    local path = SplineShape name:splineName
    addNewSpline path
    for knot in (vrigRoadKnots segments) do
        addKnot path 1 #bezierCorner #curve (toWorld knot[1] origin forward left scale) \
            (toWorld knot[2] origin forward left scale) (toWorld knot[3] origin forward left scale)
    updateShape path
    path.wirecolor = color 255 160 40
    path
)

fn vrigAddTestLabel labelName labelText location colourValue size =
(
    local label = text name:labelName text:labelText size:size
    label.position = location
    label.wirecolor = colourValue
    label.renderable = false
    label
)

-- Five realistic roads laid out in metres from `origin` along the
-- vehicle's facing, so they suit the vehicle whatever the scene scale.
-- Complex starts at the vehicle; the others run in lanes to its right.
-- Creates fixtures once and never replaces artist-authored or prior
-- evidence.
fn vrigCreateTestPaths origin forwardIndex scale =
(
    local fixtureNames = #( "VRIG_Test_Simple", "VRIG_Test_Complex", "VRIG_Test_UpHill", "VRIG_Test_DownHill", "VRIG_Test_InvalidKnot" )
    for fixtureName in fixtureNames do
        if getNodeByName fixtureName != undefined do
            return #(false, (fixtureName + " already exists; keeping the existing test evidence."), #())

    local forward = vrigAxisVector (vrigForwardAxis forwardIndex)
    local left = vrigAxisVector (vrigLeftAxis forwardIndex)
    -- #(name, lane metres to the left, segments, label, colour)
    local roads = #(
        #("VRIG_Test_Complex", 0.0, \
            #(#(#straight, 20.0), #(#arc, 30.0, -40.0), #(#arc, 30.0, 80.0), #(#arc, 30.0, -40.0), \
              #(#straight, 15.0), #(#arc, 12.0, 90.0), #(#straight, 25.0)), \
            "VALID: complex", color 80 255 120),
        #("VRIG_Test_Simple", -30.0, \
            #(#(#straight, 40.0), #(#arc, 60.0, 20.0), #(#straight, 20.0)), \
            "VALID: simple", color 80 255 120),
        #("VRIG_Test_UpHill", -45.0, \
            #(#(#straight, 15.0), #(#ramp, 40.0, 4.0), #(#straight, 15.0)), \
            "VALID: uphill", color 80 255 120),
        #("VRIG_Test_DownHill", -60.0, \
            #(#(#straight, 15.0), #(#ramp, 40.0, -4.0), #(#straight, 15.0)), \
            "VALID: downhill", color 80 255 120)
    )
    local paths = #()
    for road in roads do
    (
        local laneOrigin = origin + left * (road[2] * scale)
        append paths (vrigBuildRoad road[1] road[3] laneOrigin forward left scale)
        vrigAddTestLabel (substituteString road[1] "VRIG_Test_" "VRIG_Test_Label_") road[4] \
            (laneOrigin - forward * (4.0 * scale)) road[5] scale
    )
    -- Two coincident knots: preflight must reject this one.
    local invalidOrigin = origin + left * (-75.0 * scale)
    local invalidKnots = for knot in #([0, 0, 0], [20, 0, 0], [20, 0, 0], [40, 3, 0]) collect \
        (invalidOrigin + (forward * knot.x + left * knot.y) * scale)
    append paths (vrigBuildOpenSpline "VRIG_Test_InvalidKnot" invalidKnots false)
    vrigAddTestLabel "VRIG_Test_Label_InvalidKnot" "EXPECTED REJECT: duplicate knot" \
        (invalidOrigin - forward * (4.0 * scale)) (color 255 90 90) scale
    #(true, "Created five labelled spline test fixtures.", paths)
)

/* ------------------------------------------------------------------ *
 * Bake helpers and controllers
 * ------------------------------------------------------------------ */

-- Baked keys go on layer 1; layer 2 is zeroed and left free for manual
-- animation, so a user can offset the result without destroying the
-- generated motion. Helpers are created world-aligned, so layer 1 starts
-- at the rest position with zero rotation.
fn vrigLayeredControllers node =
(
    local restPos = in coordsys parent node.pos

    local positionList = position_list()
    node.position.controller = positionList
    while positionList.count < 2 do positionList.available.controller = Position_XYZ()
    positionList.setName 1 "Generated Path"
    positionList.setName 2 "Manual Offset"
    positionList[1].controller.value = restPos
    positionList[2].controller.value = [0, 0, 0]

    local rotationList = rotation_list()
    node.rotation.controller = rotationList
    while rotationList.count < 2 do rotationList.available.controller = Euler_XYZ()
    if classOf rotationList[1].controller != Euler_XYZ do rotationList[1].controller = Euler_XYZ()
    rotationList.setName 1 "Generated Path"
    rotationList.setName 2 "Manual Offset"
    rotationList[2].controller.value = (quat 0 0 0 1)

    positionList.setActive 2
    rotationList.setActive 2

    #(positionList[1].controller, rotationList[1].controller, restPos)
)

-- Writes one Euler track in degrees. Writing the track itself, rather than
-- a quaternion, keeps unwrapped values: a wheel at 720 degrees interpolates
-- forward to 740 instead of flipping backwards through the wrap.
fn vrigSetEulerAxis eulerController axisIndex degrees =
(
    case axisIndex of
    (
        1: eulerController.x_rotation = degrees
        2: eulerController.y_rotation = degrees
        3: eulerController.z_rotation = degrees
    )
)

fn vrigHelperPoint helperName size colourValue position parentNode =
(
    local helper = Point name:helperName size:size box:on cross:on axistripod:off \
        centermarker:off wirecolor:colourValue
    helper.transform = transMatrix position
    if parentNode != undefined do helper.parent = parentNode
    helper
)

-- Every helper sits at a measured location with world-aligned axes, so its
-- Euler tracks map directly onto the vehicle's own axes whatever the
-- source pivots look like. The body turns about `bodyPivot`, set low near
-- the roll centre so it leans like a car rather than about its axles.
fn vrigCreateHelpers measure bodyPivot =
(
    local radius = measure.tyreRadius
    local master = vrigHelperPoint "VRIG_Master" (measure.wheelbase * 0.6) (color 255 200 0) \
        measure.groundCentre undefined
    local body = vrigHelperPoint "VRIG_Body" (measure.track * 0.5) (color 120 200 255) bodyPivot master
    local steering = Circle name:"VRIG_Steering" radius:(measure.track * 0.25) wirecolor:(color 255 120 200)
    steering.renderable = false
    steering.transform = transMatrix bodyPivot
    steering.parent = master
    local pathGuide = Point name:"VRIG_PathGuide" size:(measure.wheelbase * 0.3) box:off cross:on \
        axistripod:on centermarker:off wirecolor:(color 160 255 160)
    pathGuide.transform = transMatrix measure.groundCentre
    pathGuide.parent = master

    local wheelPivots = #()
    local wheelSpins = #()
    local wheelTokens = #("FL", "FR", "RL", "RR")
    for index = 1 to 4 do
    (
        -- The circle steers; its child spins about the axle.
        local wheelPivot = Circle name:("VRIG_WheelPivot_" + wheelTokens[index]) radius:radius \
            wirecolor:(color 255 255 255)
        wheelPivot.renderable = false
        wheelPivot.transform = transMatrix measure.wheelCentres[index]
        wheelPivot.parent = master

        local wheelSpin = Point name:("VRIG_WheelSpin_" + wheelTokens[index]) size:(radius * 0.65) \
            box:off cross:on axistripod:on centermarker:on wirecolor:(color 255 190 80)
        wheelSpin.transform = transMatrix measure.wheelCentres[index]
        wheelSpin.parent = wheelPivot

        append wheelPivots wheelPivot
        append wheelSpins wheelSpin
    )
    #(master, body, steering, pathGuide, wheelPivots, wheelSpins)
)

-- A steering column riding on the body, with a child that turns about it.
-- Returns the turning child, or undefined when there is no steering wheel.
fn vrigCreateSteeringColumn steeringNode body forwardVector size =
(
    local axisInfo = vrigSteeringAxis steeringNode forwardVector
    if axisInfo == undefined then undefined
    else
    (
        local column = Point name:"VRIG_SteeringColumn" size:size box:off cross:off axistripod:on \
            centermarker:off wirecolor:(color 255 120 200)
        column.transform = vrigAxisMatrix axisInfo[2] axisInfo[1]
        column.parent = body
        local spin = Point name:"VRIG_SteeringSpin" size:(size * 0.75) box:off cross:on axistripod:off \
            centermarker:off wirecolor:(color 255 120 200)
        spin.transform = column.transform
        spin.parent = column
        spin
    )
)

-- The brake helper recorded on an assembled wheel, if any.
fn vrigBrakeFor wheelNode =
(
    local handle = getUserProp wheelNode "VRIG_brakeHandle"
    local brake = if handle == undefined or handle == 0 then undefined else maxOps.getNodeByHandle handle
    if brake != undefined and (isDeleted brake) do brake = undefined
    brake
)

/* ------------------------------------------------------------------ *
 * Baking
 * ------------------------------------------------------------------ */

fn vrigBakeAnimation settings =
(
    local profile = settings.profile
    local rootNode = settings.rootNode
    local bodyNode = settings.bodyNode
    local wheelNodes = settings.wheelNodes
    local steeringNode = settings.steeringWheelNode
    local scale = settings.scale
    local startFrame = settings.startFrame
    local endFrame = settings.endFrame

    if rootNode == undefined or not (isValidNode rootNode) do return #(false, "The vehicle root must be assigned", 1.0)
    if bodyNode == undefined or not (isValidNode bodyNode) do return #(false, "The body must be assigned", 1.0)
    for index = 1 to 4 do
        if wheelNodes[index] == undefined or not (isValidNode wheelNodes[index]) do
            return #(false, "All four wheels must be assigned", 1.0)
    if steeringNode != undefined and not (isValidNode steeringNode) do steeringNode = undefined
    if scale <= 0.0 do return #(false, "Scene units per metre must be greater than zero", 1.0)
    if profile.tyreRadius <= 0.0 do return #(false, "Tyre radius must be greater than zero", 1.0)
    if settings.gripUsage <= 0.0 or settings.gripUsage > 1.0 do
        return #(false, "Grip usage must be greater than zero and at most one", 1.0)

    -- A second bake replaces the first instead of stacking on top of it.
    vrigUnbake()

    local measured = vrigMeasureVehicle bodyNode wheelNodes
    if not measured[1] do return #(false, measured[2], 1.0)
    local measure = measured[3]
    local forwardIndex = if settings.forwardIndex == undefined then measure.forwardIndex else settings.forwardIndex
    local forwardAxis = vrigForwardAxis forwardIndex
    local leftAxis = vrigLeftAxis forwardIndex
    local forwardVector = vrigAxisVector forwardAxis
    local leftVector = vrigAxisVector leftAxis
    local forwardOffset = vrigForwardOffsetDegrees forwardIndex

    -- How far the model's own arches let the body and front wheels move.
    local brakeNodes = for wheel in wheelNodes collect (vrigBrakeFor wheel)
    local extraNodes = for brake in brakeNodes where brake != undefined collect brake
    if steeringNode != undefined do append extraNodes steeringNode
    local clearanceBody = vrigClearanceBodyNodes bodyNode wheelNodes extraNodes
    local clearance = vrigCachedClearance rootNode clearanceBody wheelNodes measure profile scale
    local steerLimits = vrigSteerLimits clearance profile

    local count = vrigSampleCountFor settings.splineNode settings.sampleCount scale
    local path = vrigMakePath (vrigSamplePath settings.splineNode count scale)
    if settings.reversed do path = vrigReversePath path
    local motion = vrigPlanMotion path profile settings.targetSpeed settings.gripUsage
    if motion.duration <= 0.0 do return #(false, "Path produced no usable motion", 1.0)

    -- The vehicle parks at the end while the body settles, so the last
    -- frame is not frozen mid-dive.
    local totalSeconds = motion.duration + settings.settleSeconds
    if settings.matchDriveTime do
        endFrame = startFrame + (amax 1 ((ceil (totalSeconds * frameRate)) as integer))
    local frameSpan = (endFrame - startFrame) as float
    if frameSpan <= 0.0 do return #(false, "End frame must be greater than start frame", 1.0)
    animationRange = interval startFrame endFrame
    -- Natural seconds per frame; 1/frameRate when the range matches the drive.
    local timeScale = totalSeconds / (frameSpan / (frameRate as float))
    local stepSeconds = timeScale / (frameRate as float)

    -- The body leans about a pivot low between the axles, near the roll
    -- centre. Corners and travel limits are in metres from that pivot.
    local bodyPivot = [measure.bodyCentre.x, measure.bodyCentre.y, \
        measure.groundZ + 0.45 * (measure.axleZ - measure.groundZ)]
    local corners = for centre in measure.wheelCentres collect \
        #((dot (centre - bodyPivot) forwardVector) / scale, (dot (centre - bodyPivot) leftVector) / scale)
    local compressionLimits = for limit in clearance.compression collect (limit / scale)
    local extensionLimit = amin profile.springTravel 0.07

    -- Wheel layout relative to the vehicle centre, in metres along the
    -- model's forward and left axes.
    local wheelForward = for centre in measure.wheelCentres collect ((dot (centre - measure.groundCentre) forwardVector) / scale)
    local wheelLeft = for centre in measure.wheelCentres collect ((dot (centre - measure.groundCentre) leftVector) / scale)

    local helpers = vrigCreateHelpers measure bodyPivot
    local master = helpers[1]
    local body = helpers[2]
    local wheelPivots = helpers[5]
    local wheelSpins = helpers[6]
    local steeringSpin = if steeringNode == undefined then undefined \
        else vrigCreateSteeringColumn steeringNode body forwardVector (measure.tyreRadius * 0.8)

    -- Bind while every generated helper is still at its rest pose.
    -- Parenting after the animated key loop would bake an inverse offset
    -- from the final pose into the source meshes.
    vrigRecordOriginalState rootNode "bake"
    rootNode.parent = master
    local bodyDriven = bodyNode != rootNode
    if bodyDriven do
    (
        vrigRecordOriginalState bodyNode "bake"
        bodyNode.parent = body
    )
    for index = 1 to 4 do
    (
        vrigRecordOriginalState wheelNodes[index] "bake"
        wheelNodes[index].parent = wheelSpins[index]
        if brakeNodes[index] != undefined do
        (
            vrigRecordOriginalState brakeNodes[index] "bake"
            brakeNodes[index].parent = wheelPivots[index]
        )
    )
    if steeringSpin != undefined do
    (
        vrigRecordOriginalState steeringNode "bake"
        steeringNode.parent = steeringSpin
    )

    local masterControllers = vrigLayeredControllers master
    local bodyControllers = vrigLayeredControllers body
    local pivotControllers = for wheelPivot in wheelPivots collect (vrigLayeredControllers wheelPivot)
    local spinControllers = for wheelSpin in wheelSpins collect (vrigLayeredControllers wheelSpin)
    local steeringControllers = if steeringSpin == undefined then undefined else (vrigLayeredControllers steeringSpin)

    local heaveState = #(0.0, 0.0)
    local pitchState = #(0.0, 0.0)
    local rollState = #(0.0, 0.0)
    local wheelSpin = #(0.0, 0.0, 0.0, 0.0)
    local previousWheel = undefined
    local previousSpeed = undefined
    local previousYaw = undefined
    local peakSpeed = 0.0
    local peakSteer = 0.0
    local peakRoll = 0.0
    local peakPitch = 0.0
    local peakCompression = 0.0
    local steerHeld = false
    local leanHeld = false

    with animate on
    (
        for frame = startFrame to endFrame do
        (
            local elapsed = ((frame - startFrame) as float) * stepSeconds
            local state = vrigStateAtTime motion elapsed
            local travelled = state[1]
            local speed = state[2]
            local curvature = state[3]
            local pose = vrigAxlePose path travelled profile.wheelbase
            local heading = pose[2]

            local yaw = heading + forwardOffset
            if previousYaw != undefined do yaw = vrigUnwrapDegrees yaw previousYaw
            previousYaw = yaw

            -- Wheel contact points, for per-wheel rolling distance.
            local headingForward = [cos heading, sin heading, 0.0]
            local headingLeft = [-(sin heading), cos heading, 0.0]
            local wheelNow = for index = 1 to 4 collect \
                ((vrigPathPoint path (travelled + wheelForward[index])) + headingLeft * wheelLeft[index])

            -- Body: the settled pose for these loads, fitted inside the
            -- arches, then eased toward by the springs and fitted again in
            -- case the spring overshoots.
            local longitudinal = if previousSpeed == undefined or stepSeconds <= 1e-9 then 0.0 \
                else ((speed - previousSpeed) / stepSeconds)
            local lateral = curvature * (vrigSquare speed)
            local vertical = state[4] * (vrigSquare speed)
            local target = vrigBodyResponse profile longitudinal lateral vertical \
                settings.rollIntensity settings.accelIntensity settings.brakeIntensity settings.suspIntensity
            local wanted = vrigFitBodyPose target[3] target[2] (-target[1]) corners compressionLimits extensionLimit
            if (abs (wanted[2] - target[2])) > 0.01 or (abs (wanted[3] + target[1])) > 0.01 or \
                (wanted[1] - target[3]) > 0.0005 do leanHeld = true
            if previousSpeed != undefined do
            (
                heaveState = vrigSuspensionStep heaveState wanted[1] stepSeconds profile.rideFrequency profile.dampingRatio
                pitchState = vrigSuspensionStep pitchState wanted[2] stepSeconds profile.rideFrequency profile.dampingRatio
                rollState = vrigSuspensionStep rollState wanted[3] stepSeconds profile.rideFrequency profile.dampingRatio
            )
            local bodyPose = vrigFitBodyPose heaveState[1] pitchState[1] rollState[1] corners compressionLimits extensionLimit
            for corner in corners do
                peakCompression = amax peakCompression (-(vrigCornerRise bodyPose[1] bodyPose[2] bodyPose[3] corner))

            -- Steering, held at the lock the arches allow.
            local wantedSteer = vrigSteeringAngle curvature speed profile
            local steerCentre = if wantedSteer >= 0.0 then (amin wantedSteer steerLimits[1]) \
                else (amax wantedSteer (-steerLimits[2]))
            if (abs (steerCentre - wantedSteer)) > 0.05 do steerHeld = true
            local steerAngles = vrigAckermannAngles steerCentre profile.wheelbase profile.track 90.0
            -- Positive steer is a left turn, which makes the left wheel the inner one.
            local steerLeft = if steerCentre >= 0.0 then steerAngles[1] else steerAngles[2]
            local steerRight = if steerCentre >= 0.0 then steerAngles[2] else steerAngles[1]

            at time frame
            (
                masterControllers[1].value = pose[1] * scale
                -- Pitch about the model's left axis first, then yaw.
                vrigSetEulerAxis masterControllers[2] leftAxis[1] (-1.0 * leftAxis[2] * pose[3])
                vrigSetEulerAxis masterControllers[2] 3 yaw

                if bodyDriven do
                (
                    -- Roll lifts the left side about the forward axis; pitch is nose-up.
                    bodyControllers[1].value = bodyControllers[3] + [0, 0, bodyPose[1] * scale]
                    vrigSetEulerAxis bodyControllers[2] forwardAxis[1] (forwardAxis[2] * bodyPose[3])
                    vrigSetEulerAxis bodyControllers[2] leftAxis[1] (-1.0 * leftAxis[2] * bodyPose[2])
                )

                for index = 1 to 4 do
                (
                    if previousWheel != undefined do
                    (
                        local moved = wheelNow[index] - previousWheel[index]
                        local rolled = length moved
                        if (dot moved headingForward) < 0.0 do rolled = -rolled
                        wheelSpin[index] += radToDeg (rolled / profile.tyreRadius)
                    )
                    vrigSetEulerAxis spinControllers[index][2] leftAxis[1] (leftAxis[2] * wheelSpin[index])
                )
                vrigSetEulerAxis pivotControllers[1][2] 3 steerLeft
                vrigSetEulerAxis pivotControllers[2][2] 3 steerRight
                -- A left turn turns the wheel anticlockwise as the driver sees it.
                if steeringControllers != undefined do
                    vrigSetEulerAxis steeringControllers[2] 3 (steerCentre * profile.steeringRatio)
            )

            previousWheel = wheelNow
            previousSpeed = speed
            peakSpeed = amax peakSpeed speed
            peakSteer = amax peakSteer (amax (abs steerLeft) (abs steerRight))
            peakRoll = amax peakRoll (abs bodyPose[3])
            peakPitch = amax peakPitch (abs bodyPose[2])
        )
    )

    local notes = #()
    append notes ("Frames " + (startFrame as string) + " to " + (endFrame as string) + "; drive " + \
        (formattedPrint motion.duration format:".1f") + " s plus " + \
        (formattedPrint settings.settleSeconds format:".1f") + " s to settle, top speed " + \
        (formattedPrint (peakSpeed * 3.6) format:".0f") + " km/h.")
    append notes ("Peak wheel angle " + (formattedPrint peakSteer format:".1f") + " deg, body roll " + \
        (formattedPrint peakRoll format:".1f") + " deg, pitch " + (formattedPrint peakPitch format:".1f") + \
        " deg; no corner compresses more than " + (formattedPrint (peakCompression * 100.0) format:".1f") + " cm.")
    append notes (vrigDescribeClearance clearance profile scale)
    if steerHeld do append notes "In the tightest sections the front wheels stop at the lock the arches allow."
    if leanHeld do append notes "Where the full lean would reach an arch, the body rises or leans less instead."
    if not bodyDriven do append notes "The body is the vehicle root, so body roll and pitch are not applied."
    if steeringNode != undefined and steeringSpin == undefined do
        append notes "The steering wheel has no geometry to measure, so it does not turn."
    if timeScale > 1.25 do
        append notes ("The frame range is shorter than the drive; motion plays " + \
            (formattedPrint timeScale format:".2f") + "x faster than the physics.")
    if timeScale < 0.8 do
        append notes ("The frame range is longer than the drive; motion plays at " + \
            (formattedPrint timeScale format:".2f") + "x speed.")
    local summary = notes[1]
    for index = 2 to notes.count do summary += " " + notes[index]
    #(true, summary, timeScale, endFrame)
)

/* ------------------------------------------------------------------ *
 * Dialogue
 * ------------------------------------------------------------------ */

-- Share of the tyres' grip each driving style uses.
fn vrigDrivingStyles = #(#("Relaxed", 0.35), #("Normal", 0.55), #("Sporty", 0.75), #("Limit", 1.0))

rollout VehicleRigRollout "Create Vehicle Rig" width:360
(
    local boundRoot = undefined
    local boundBody = undefined
    local boundWheels = #(undefined, undefined, undefined, undefined)
    local boundSteeringWheel = undefined
    local boundSpline = undefined
    local measure = undefined
    local clearance = undefined
    local forwardOverride = false
    local preflightPassed = false
    local acknowledgedWarnings = false

    group "Vehicle Rig"
    (
        button autoAssembleButton "Auto Assemble Imported Vehicle" width:320
        pickbutton rootPick "Pick Vehicle Root" width:160 across:2 message:"Pick the vehicle root"
        label rootLabel "none" align:#left
        pickbutton bodyPick "Pick Body" width:160 across:2 message:"Pick the body"
        label bodyLabel "none" align:#left
        pickbutton wheelFLPick "Pick Wheel FL" width:160 across:2 message:"Pick front-left wheel"
        label wheelFLLabel "none" align:#left
        pickbutton wheelFRPick "Pick Wheel FR" width:160 across:2 message:"Pick front-right wheel"
        label wheelFRLabel "none" align:#left
        pickbutton wheelRLPick "Pick Wheel RL" width:160 across:2 message:"Pick rear-left wheel"
        label wheelRLLabel "none" align:#left
        pickbutton wheelRRPick "Pick Wheel RR" width:160 across:2 message:"Pick rear-right wheel"
        label wheelRRLabel "none" align:#left
        pickbutton steeringPick "Pick Steering Wheel" width:160 across:2 message:"Pick the steering wheel (optional)"
        label steeringLabel "none (optional)" align:#left
        dropdownlist presetList "Vehicle preset" items:#("car") width:160
        spinner tyreRadiusSpin "Tyre radius (m)" range:[0.05, 1.5, 0.31] type:#float scale:0.01
        -- Filled from the measured wheels. A model built at a different
        -- scale from the scene units is fitted to the preset wheelbase.
        spinner unitScaleSpin "Scene units per metre" range:[1e-6, 1e6, 1.0] type:#float scale:0.1
        dropdownlist forwardAxisList "Vehicle faces" items:#("+X", "-X", "+Y", "-Y") width:160
    )

    group "Path and Motion"
    (
        pickbutton splinePick "Pick Open Spline" width:160 across:2 message:"Pick one open spline"
        label splineLabel "none" align:#left
        dropdownlist styleList "Driving style" items:#("Normal") width:160
        spinner startFrameSpin "Start frame" range:[-100000, 100000, 0] type:#integer
        spinner endFrameSpin "End frame" range:[-100000, 100000, 100] type:#integer
        checkbox matchDriveCheck "Match frame range to drive time" checked:true
        checkbox reverseCheck "Travel in reverse" checked:false
        spinner targetSpeedSpin "Target speed (m/s)" range:[0.1, 200.0, 30.0] type:#float scale:0.5
        spinner sampleSpin "Path samples" range:[16, 4096, 256] type:#integer
        spinner rollSpin "Body roll" range:[0.0, 1.0, 1.0] type:#float scale:0.05
        spinner accelSpin "Acceleration pitch" range:[0.0, 1.0, 1.0] type:#float scale:0.05
        spinner brakeSpin "Braking pitch" range:[0.0, 1.0, 1.0] type:#float scale:0.05
        spinner suspensionSpin "Suspension" range:[0.0, 1.0, 1.0] type:#float scale:0.05
    )

    group "Actions"
    (
        button createTestsButton "Create Five Spline Tests" width:160
        button preflightButton "Preflight" width:160 across:2
        checkbox acknowledgeCheck "Accept warnings" checked:false enabled:false
        button bakeButton "Bake Animation" width:160 enabled:false
        button removeButton "Remove Generated Rig" width:160
    )

    edittext reportBox "" height:170 width:340 readonly:true

    fn report lines =
    (
        local text = ""
        for line in lines do text += line + "\r\n"
        reportBox.text = text
    )

    fn selectedGripUsage = ((vrigDrivingStyles())[styleList.selection])[2]

    fn activeProfile =
    (
        local preset = vrigPresetByName (presetList.selected)
        if preset == undefined then undefined
        else if measure != undefined then
            vrigMeasuredProfile preset measure unitScaleSpin.value tyreRadiusSpin.value
        else
        (
            local copy = vrigCopyProfile preset
            copy.tyreRadius = tyreRadiusSpin.value
            copy
        )
    )

    fn invalidateBake =
    (
        preflightPassed = false
        acknowledgedWarnings = false
        acknowledgeCheck.checked = false
        acknowledgeCheck.enabled = false
        bakeButton.enabled = false
    )

    fn wheelsBound =
    (
        local complete = boundBody != undefined and boundRoot != undefined
        for wheel in boundWheels where wheel == undefined do complete = false
        complete
    )

    -- Measure once the root, body and four wheels are known: fill the
    -- scale, tyre radius and facing from the model and measure how far its
    -- wheel arches let the body move. Returns report lines.
    fn applyMeasurement =
    (
        measure = undefined
        clearance = undefined
        if not (wheelsBound()) then #()
        else
        (
            local result = vrigMeasureVehicle boundBody boundWheels
            if not result[1] then #("Measurement failed: " + result[2])
            else
            (
                measure = result[3]
                local preset = vrigPresetByName (presetList.selected)
                local fitted = vrigSuggestedScale measure (vrigUnitScale()) preset.wheelbase
                unitScaleSpin.value = fitted[1]
                tyreRadiusSpin.value = measure.tyreRadius / fitted[1]
                forwardAxisList.selection = measure.forwardIndex
                forwardOverride = false
                local lines = #(vrigDescribeMeasure measure fitted[1])
                if fitted[2] do
                    append lines ("The scene says " + (formattedPrint (vrigUnitScale()) format:".3f") + \
                        " units/m, which would make this wheelbase " + \
                        (vrigMetres (measure.wheelbase / (vrigUnitScale()))) + \
                        ". Fitted the scale to the " + preset.name + " preset instead; adjust if the model differs.")
                -- A baked vehicle is not at rest; the bake measures it after unbaking.
                if (vrigBakeHelpers()).count == 0 then
                (
                    local extras = for wheel in boundWheels collect (vrigBrakeFor wheel)
                    extras = for node in extras where node != undefined collect node
                    if boundSteeringWheel != undefined do append extras boundSteeringWheel
                    local bodyNodes = vrigClearanceBodyNodes boundBody boundWheels extras
                    clearance = vrigCachedClearance boundRoot bodyNodes boundWheels measure (activeProfile()) fitted[1]
                    append lines (vrigDescribeClearance clearance (activeProfile()) fitted[1])
                )
                else append lines "Wheel-arch clearance is measured when you bake."
                lines
            )
        )
    )

    fn bindingErrors =
    (
        local errors = #()
        if boundRoot == undefined do append errors "Pick the vehicle root"
        if boundBody == undefined do append errors "Pick the body"
        for index = 1 to 4 do
            if boundWheels[index] == undefined do
                append errors ("Pick wheel " + (#("FL", "FR", "RL", "RR"))[index])
        for node in #(boundRoot, boundBody, boundWheels[1], boundWheels[2], boundWheels[3], boundWheels[4], boundSteeringWheel) \
            where node != undefined and not (isValidNode node) do
            append errors "A picked node has been deleted; pick it again"
        if boundRoot != undefined and (isValidNode boundRoot) do
        (
            local assigned = #(boundBody, boundWheels[1], boundWheels[2], boundWheels[3], boundWheels[4], boundSteeringWheel)
            for node in assigned where node != undefined and (isValidNode node) do
                if not (vrigIsDescendantOf node boundRoot) do
                    append errors (node.name + " is outside the selected vehicle hierarchy")
        )
        if tyreRadiusSpin.value <= 0.0 do append errors "Tyre radius must be greater than zero"
        errors
    )

    fn bindPicked =
    (
        rootLabel.text = if boundRoot == undefined then "none" else boundRoot.name
        bodyLabel.text = if boundBody == undefined then "none" else boundBody.name
        steeringLabel.text = if boundSteeringWheel == undefined then "none (optional)" else boundSteeringWheel.name
        local labels = #(wheelFLLabel, wheelFRLabel, wheelRLLabel, wheelRRLabel)
        for index = 1 to 4 do
            labels[index].text = if boundWheels[index] == undefined then "none" else boundWheels[index].name
        invalidateBake()
        applyMeasurement()
    )

    fn reportPick =
    (
        local lines = bindPicked()
        if lines.count > 0 do report lines
    )

    on VehicleRigRollout open do
    (
        presetList.items = vrigPresetNames()
        presetList.selection = 1
        styleList.items = for style in (vrigDrivingStyles()) collect style[1]
        styleList.selection = 2
        startFrameSpin.value = animationRange.start.frame as integer
        endFrameSpin.value = animationRange.end.frame as integer
        local profile = vrigPresetByName (presetList.selected)
        if profile != undefined do tyreRadiusSpin.value = profile.tyreRadius
        unitScaleSpin.value = vrigUnitScale()
        forwardAxisList.selection = 1
        report #("Select an imported vehicle and press Auto Assemble, or pick the root, body and four wheels.", \
            "Then pick one open spline and run Preflight.")
    )

    on presetList selected index do
    (
        -- A measured tyre keeps its size; only an unmeasured rig takes the preset's.
        local profile = vrigPresetByName (presetList.selected)
        if profile != undefined and measure == undefined do tyreRadiusSpin.value = profile.tyreRadius
        invalidateBake()
    )

    on autoAssembleButton pressed do
    (
        local vehicleName = "ImportedVehicle"
        if selection.count > 0 do
        (
            local nameParts = filterString selection[1].name ":"
            if nameParts.count > 0 do vehicleName = nameParts[1]
        )
        local result = undefined
        undo "Assemble Vehicle" on ( result = vrigAssembleSelectedVehicle vehicleName )
        if result[1] then
        (
            boundRoot = result[3]
            boundBody = result[4]
            boundWheels = result[5]
            boundSteeringWheel = result[7]
            local lines = #(result[2])
            join lines (bindPicked())
            append lines "Pick an open spline, or press Create Five Spline Tests, then run Preflight."
            report lines
        )
        else report #("Assembly failed:", result[2])
    )

    on createTestsButton pressed do
    (
        local origin = if measure != undefined then measure.groundCentre else [0, 0, 0]
        local result = undefined
        undo "Create Spline Tests" on
            ( result = vrigCreateTestPaths origin forwardAxisList.selection unitScaleSpin.value )
        if result[1] then report #(result[2], "Simple, Complex, UpHill and DownHill are VALID. InvalidKnot is EXPECTED REJECT.")
        else report #("Test-path creation skipped:", result[2])
    )

    on rootPick picked node do ( boundRoot = node; reportPick() )
    on bodyPick picked node do ( boundBody = node; reportPick() )
    on wheelFLPick picked node do ( boundWheels[1] = node; reportPick() )
    on wheelFRPick picked node do ( boundWheels[2] = node; reportPick() )
    on wheelRLPick picked node do ( boundWheels[3] = node; reportPick() )
    on wheelRRPick picked node do ( boundWheels[4] = node; reportPick() )
    on steeringPick picked node do ( boundSteeringWheel = node; reportPick() )
    on splinePick picked node do ( boundSpline = node; splineLabel.text = node.name; invalidateBake() )

    on tyreRadiusSpin changed value do invalidateBake()
    on unitScaleSpin changed value do invalidateBake()
    on forwardAxisList selected index do ( forwardOverride = true; invalidateBake() )
    on styleList selected index do invalidateBake()
    on reverseCheck changed state do invalidateBake()
    on matchDriveCheck changed state do invalidateBake()
    on startFrameSpin changed value do invalidateBake()
    on endFrameSpin changed value do invalidateBake()
    on sampleSpin changed value do invalidateBake()
    on targetSpeedSpin changed value do invalidateBake()

    on acknowledgeCheck changed state do
    (
        acknowledgedWarnings = state
        bakeButton.enabled = preflightPassed and state
    )

    -- Preflight never touches the scene.
    on preflightButton pressed do
    (
        invalidateBake()
        local lines = #()
        local profile = activeProfile()
        local errors = bindingErrors()
        join errors (vrigValidateProfile profile)
        join errors (vrigValidateSplineNode boundSpline)
        if errors.count == 0 and measure == undefined do
            append errors "The vehicle could not be measured; re-pick the body and wheels"

        if errors.count > 0 then
        (
            append lines "Preflight failed:"
            join lines errors
            report lines
        )
        else
        (
            local scale = unitScaleSpin.value
            local steerLimits = if clearance == undefined then undefined else (vrigSteerLimits clearance profile)
            local checked = vrigPreflightPath boundSpline profile scale sampleSpin.value \
                targetSpeedSpin.value (measure.length / scale) (selectedGripUsage()) steerLimits
            local issues = checked[1]
            local motion = checked[2]
            local errorCount = vrigCountBySeverity issues "error"
            local warningCount = vrigCountBySeverity issues "warning"
            append lines (vrigDescribeMeasure measure scale)

            if errorCount > 0 then
            (
                append lines ("Preflight failed with " + (errorCount as string) + " error(s):")
                join lines (vrigFormatIssues issues)
            )
            else
            (
                local settle = (VehicleRigBakeSettings()).settleSeconds
                local frames = (ceil ((motion.duration + settle) * frameRate)) as integer
                local peak = 0.0
                for speed in motion.speeds do peak = amax peak speed
                append lines ("Path " + (vrigMetres motion.path.length) + "; " + styleList.selected + \
                    " driving takes " + (formattedPrint motion.duration format:".1f") + " s at up to " + \
                    (formattedPrint (peak * 3.6) format:".0f") + " km/h, plus " + \
                    (formattedPrint settle format:".1f") + " s to settle.")
                if matchDriveCheck.checked then
                    append lines ("Bake will use frames " + (startFrameSpin.value as string) + " to " + \
                        ((startFrameSpin.value + frames) as string) + ".")
                else
                    append lines ("Bake will fit the drive into frames " + (startFrameSpin.value as string) + \
                        " to " + (endFrameSpin.value as string) + ".")
                preflightPassed = true
                acknowledgeCheck.enabled = true
                if warningCount > 0 then
                (
                    append lines ("Preflight passed with " + (warningCount as string) + " warning(s).")
                    append lines "Review each warning, then tick Accept warnings to enable baking."
                    join lines (vrigFormatIssues issues)
                )
                else
                (
                    acknowledgeCheck.checked = true
                    acknowledgedWarnings = true
                    bakeButton.enabled = true
                    append lines "Preflight passed with no issues."
                )
            )
            report lines
        )
    )

    on bakeButton pressed do
    (
        if not (preflightPassed and acknowledgedWarnings) then
            report #("Run Preflight and accept any warnings before baking.")
        else
        (
            local settings = VehicleRigBakeSettings profile:(activeProfile()) splineNode:boundSpline \
                rootNode:boundRoot bodyNode:boundBody wheelNodes:boundWheels \
                steeringWheelNode:boundSteeringWheel \
                startFrame:startFrameSpin.value endFrame:endFrameSpin.value \
                reversed:reverseCheck.checked targetSpeed:targetSpeedSpin.value \
                rollIntensity:rollSpin.value accelIntensity:accelSpin.value \
                brakeIntensity:brakeSpin.value suspIntensity:suspensionSpin.value \
                sampleCount:sampleSpin.value scale:unitScaleSpin.value \
                forwardIndex:(if forwardOverride then forwardAxisList.selection else undefined) \
                matchDriveTime:matchDriveCheck.checked gripUsage:(selectedGripUsage())
            local result = undefined
            undo "Bake Vehicle Rig" on ( result = vrigBakeAnimation settings )
            if result[1] then
            (
                endFrameSpin.value = result[4]
                report #("Bake complete.", result[2], \
                    "Manual Offset is the active controller layer; edit it without losing the baked path.")
            )
            else report #("Bake failed:", result[2])
            invalidateBake()
        )
    )

    on removeButton pressed do
    (
        local result = undefined
        undo "Remove Vehicle Rig" on ( result = vrigRemoveGeneratedRig() )
        boundRoot = undefined
        boundBody = undefined
        boundWheels = #(undefined, undefined, undefined, undefined)
        boundSteeringWheel = undefined
        measure = undefined
        clearance = undefined
        bindPicked()
        report #(
            "Removed " + (result[2] as string) + " generated node(s).",
            "Restored " + (result[1] as string) + " node(s) to their original parent and transform."
        )
    )
)

if VehicleRigFloater != undefined and VehicleRigFloater != false do
    closeRolloutFloater VehicleRigFloater
global VehicleRigFloater = newRolloutFloater "Vehicle Rig" 380 900
addRollout VehicleRigRollout VehicleRigFloater
