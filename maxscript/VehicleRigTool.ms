/*
    VehicleRigTool.ms
    Native 3ds Max vehicle rig and path motion tool.

    Binds one existing vehicle and one open spline, validates the path, and
    bakes editable keys onto generated VRIG_ helpers. Source geometry and
    existing controllers are left intact; every bound node records its
    original parent and transform so the rig can be removed cleanly.

    Parameter names were written against 3ds Max 2027. Treat other versions
    as unverified until checked on the host.

    Out of scope by design: file reads and writes, network access, terrain,
    lighting, cameras, and rendering.
*/

struct VehicleRigProfile
(
    name, wheelbase, track, tyreRadius, topSpeed, acceleration, braking,
    lateralGrip, maxSteer, springTravel, dampingRatio, maxRoll, maxPitch
)

struct VehicleRigIssue (code, severity, message, index)

/* ------------------------------------------------------------------ *
 * Vehicle presets. These mirror server/vehicle_dynamics.py exactly;
 * tests/test_vehicle_rig_script.py fails if the two ever drift.
 * ------------------------------------------------------------------ */

fn vrigPresets =
(
    #(
        VehicleRigProfile name:"car" wheelbase:2.70 track:1.55 tyreRadius:0.31 topSpeed:70.0 acceleration:5.5 braking:9.0 lateralGrip:1.00 maxSteer:35.0 springTravel:0.12 dampingRatio:0.70 maxRoll:6.0 maxPitch:4.0,
        VehicleRigProfile name:"suv" wheelbase:2.85 track:1.65 tyreRadius:0.36 topSpeed:55.0 acceleration:4.0 braking:8.0 lateralGrip:0.85 maxSteer:32.0 springTravel:0.18 dampingRatio:0.75 maxRoll:9.0 maxPitch:5.0,
        VehicleRigProfile name:"pickup" wheelbase:3.20 track:1.72 tyreRadius:0.38 topSpeed:48.0 acceleration:3.2 braking:7.0 lateralGrip:0.75 maxSteer:30.0 springTravel:0.20 dampingRatio:0.80 maxRoll:10.0 maxPitch:5.5,
        VehicleRigProfile name:"van" wheelbase:3.10 track:1.70 tyreRadius:0.35 topSpeed:42.0 acceleration:3.0 braking:6.5 lateralGrip:0.70 maxSteer:28.0 springTravel:0.18 dampingRatio:0.80 maxRoll:10.0 maxPitch:5.0,
        VehicleRigProfile name:"bus" wheelbase:6.00 track:2.35 tyreRadius:0.49 topSpeed:30.0 acceleration:1.5 braking:5.0 lateralGrip:0.55 maxSteer:24.0 springTravel:0.16 dampingRatio:0.85 maxRoll:7.0 maxPitch:3.5
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
        maxRoll:profile.maxRoll maxPitch:profile.maxPitch
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
        #("max pitch", profile.maxPitch)
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

fn vrigWheelRotationDegrees distance tyreRadius =
(
    if tyreRadius <= 0.0 then 0.0 else radToDeg (distance / tyreRadius)
)

fn vrigBodyResponse profile longitudinal lateral rollIntensity accelIntensity brakeIntensity suspIntensity =
(
    local lateralLimit = (vrigGravity()) * profile.lateralGrip
    local roll = vrigClamp (-lateral / lateralLimit * profile.maxRoll * rollIntensity) profile.maxRoll
    local longLimit = if longitudinal >= 0.0 then profile.acceleration else profile.braking
    local longIntensity = if longitudinal >= 0.0 then accelIntensity else brakeIntensity
    local pitch = vrigClamp (longitudinal / longLimit * profile.maxPitch * longIntensity) profile.maxPitch
    local loadRatio = amin 1.0 (sqrt ((vrigSquare (longitudinal / longLimit)) + (vrigSquare (lateral / lateralLimit))))
    local heave = -loadRatio * profile.springTravel * suspIntensity
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
        local location = if issue.index == undefined then "" else (" at point " + (issue.index as string))
        append lines ("[" + issue.severity + "] " + issue.code + location + ": " + issue.message)
    )
    lines
)

/* ------------------------------------------------------------------ *
 * Spline sampling
 * ------------------------------------------------------------------ */

fn vrigValidateSplineNode splineNode =
(
    local errors = #()
    if splineNode == undefined then append errors "No spline selected"
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

-- Samples the spline by arc length and returns world-space points in metres.
-- `scale` is scene units per metre; see vrigUnitScale for the default.
fn vrigSamplePath splineNode sampleCount scale =
(
    local offset = splineNode.objecttransform
    local points = #()
    -- lengthInterp can clamp its final interval before the actual last knot.
    -- Sample the interpolated interior only, then append the exact endpoint.
    for index = 0 to sampleCount - 1 do
    (
        local param = (index as float) / (sampleCount as float)
        local local_point = lengthInterp splineNode 1 param
        local world_point = local_point * offset
        append points [world_point.x / scale, world_point.y / scale, world_point.z / scale]
    )
    local finalLocalPoint = getKnotPoint splineNode 1 (numKnots splineNode 1)
    local finalWorldPoint = finalLocalPoint * offset
    append points [finalWorldPoint.x / scale, finalWorldPoint.y / scale, finalWorldPoint.z / scale]
    points
)

/* ------------------------------------------------------------------ *
 * Motion solve: two-pass grip-limited speed profile
 * ------------------------------------------------------------------ */

fn vrigCumulativeDistances points =
(
    local distances = #(0.0)
    for index = 1 to points.count - 1 do
        append distances (distances[index] + (distance points[index] points[index + 1]))
    distances
)

-- Signed curvature per sample, positive for a left turn.
fn vrigCurvatures points =
(
    local curvatures = #()
    append curvatures 0.0
    for index = 2 to points.count - 1 do
    (
        local result = vrigCurvatureRadius2D points[index - 1] points[index] points[index + 1]
        local radius = result[1]
        local value = if radius == undefined or radius <= 0.0 then 0.0 else (1.0 / radius)
        local turn = vrigOrientation points[index - 1] points[index] points[index + 1]
        if turn < 0.0 do value = -value
        append curvatures value
    )
    append curvatures 0.0
    curvatures
)

fn vrigSpeedProfile points distances curvatures profile targetSpeed =
(
    local count = points.count
    local ceiling = amin targetSpeed profile.topSpeed
    local speeds = #()
    for index = 1 to count do
    (
        local radius = if (abs curvatures[index]) <= 1e-9 then 1e9 else (1.0 / (abs curvatures[index]))
        append speeds (amin ceiling (vrigCurveSpeedLimit radius profile))
    )

    -- The vehicle starts and finishes at rest.
    speeds[1] = 0.0
    speeds[count] = 0.0

    for index = 2 to count do
    (
        local span = distances[index] - distances[index - 1]
        local reachable = sqrt ((vrigSquare speeds[index - 1]) + 2.0 * profile.acceleration * span)
        speeds[index] = amin speeds[index] reachable
    )
    for index = count - 1 to 1 by -1 do
    (
        local span = distances[index + 1] - distances[index]
        local reachable = sqrt ((vrigSquare speeds[index + 1]) + 2.0 * profile.braking * span)
        speeds[index] = amin speeds[index] reachable
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

-- Linear search for the sample bracketing a travel time, then interpolate.
fn vrigSampleAtTime times distances speeds curvatures points elapsed =
(
    local count = times.count
    if elapsed <= times[1] then #(distances[1], speeds[1], curvatures[1], 1, 0.0)
    else if elapsed >= times[count] then #(distances[count], speeds[count], curvatures[count], count, 0.0)
    else
    (
        local index = 1
        for probe = 1 to count - 1 while times[probe + 1] <= elapsed do index = probe + 1
        local span = times[index + 1] - times[index]
        local blend = if span <= 1e-9 then 0.0 else ((elapsed - times[index]) / span)
        local travelled = distances[index] + blend * (distances[index + 1] - distances[index])
        local speed = speeds[index] + blend * (speeds[index + 1] - speeds[index])
        local curvature = curvatures[index] + blend * (curvatures[index + 1] - curvatures[index])
        #(travelled, speed, curvature, index, blend)
    )
)

fn vrigPointAtDistance points distances travelled =
(
    local count = points.count
    if travelled <= distances[1] then points[1]
    else if travelled >= distances[count] then points[count]
    else
    (
        local index = 1
        for probe = 1 to count - 1 while distances[probe + 1] <= travelled do index = probe + 1
        local span = distances[index + 1] - distances[index]
        local blend = if span <= 1e-9 then 0.0 else ((travelled - distances[index]) / span)
        points[index] + blend * (points[index + 1] - points[index])
    )
)

-- Reverse travel is handled by flipping the sampled path, so the heading
-- here is always the direction of travel.
fn vrigHeadingAtDistance points distances travelled =
(
    local count = points.count
    local index = 1
    for probe = 1 to count - 1 while distances[probe + 1] <= travelled do index = probe + 1
    if index >= count do index = count - 1
    local forward = points[index + 1] - points[index]
    if (length forward) <= 1e-9 then 0.0 else (atan2 forward.y forward.x)
)

/* ------------------------------------------------------------------ *
 * Restore records. User properties keep the data host-portable and
 * survive a save, unlike an in-memory table.
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

fn vrigRecordOriginalState node =
(
    local originalParent = if node.parent == undefined then "" else node.parent.name
    local originalTransform = vrigMatrixToString node.transform
    setUserProp node "VRIG_originalParent" originalParent
    setUserProp node "VRIG_originalTransform" originalTransform
    setUserProp node "VRIG_bound" "true"
)

fn vrigRestoreOriginalState node =
(
    if (getUserProp node "VRIG_bound") == undefined then false
    else
    (
        local originalTransform = getUserProp node "VRIG_originalTransform"
        local originalParent = getUserProp node "VRIG_originalParent"
        local restored = if originalTransform == undefined then undefined \
            else vrigStringToMatrix (originalTransform as string)

        local parentNode = undefined
        if originalParent != undefined and (originalParent as string) != "" do
            parentNode = getNodeByName (originalParent as string)
        node.parent = parentNode
        if restored != undefined do node.transform = restored

        setUserProp node "VRIG_bound" undefined
        setUserProp node "VRIG_originalParent" undefined
        setUserProp node "VRIG_originalTransform" undefined
        true
    )
)

fn vrigBoundNodes =
(
    for node in objects where (getUserProp node "VRIG_bound") != undefined collect node
)

fn vrigGeneratedNodes =
(
    for node in objects where ((matchPattern node.name pattern:"VRIG_*") and not (matchPattern node.name pattern:"VRIG_Test_*")) collect node
)

fn vrigRemoveGeneratedRig =
(
    local restoredCount = 0
    for node in (vrigBoundNodes()) do
        if vrigRestoreOriginalState node do restoredCount += 1

    local generated = vrigGeneratedNodes()
    local generatedCount = generated.count
    if generatedCount > 0 do delete generated

    #(restoredCount, generatedCount)
)

/* ------------------------------------------------------------------ *
 * Rig construction
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

fn vrigBoundsCenter nodes =
(
    local firstBounds = nodeGetBoundingBox nodes[1] nodes[1].transform
    local low = firstBounds[1]
    local high = firstBounds[2]
    for index = 2 to nodes.count do
    (
        local bounds = nodeGetBoundingBox nodes[index] nodes[index].transform
        low = [amin low.x bounds[1].x, amin low.y bounds[1].y, amin low.z bounds[1].z]
        high = [amax high.x bounds[2].x, amax high.y bounds[2].y, amax high.z bounds[2].z]
    )
    (low + high) / 2.0
)

fn vrigAssembleSelectedVehicle vehicleName =
(
    local sourceNodes = for node in selection where (superClassOf node == GeometryClass) collect node
    if sourceNodes.count == 0 do return #(false, "Select the imported vehicle mesh nodes before assembly.", undefined, undefined, #())

    local wheelSets = #(#(), #(), #(), #())
    for node in sourceNodes do
    (
        if (vrigWheelMatchCount node) > 1 do
            return #(false, (node.name + " matches more than one wheel cluster."), undefined, undefined, #())
        local wheelIndex = vrigClassifyWheelNode node
        if wheelIndex > 0 do append wheelSets[wheelIndex] node
    )
    for wheelIndex = 1 to 4 do
        if wheelSets[wheelIndex].count == 0 do
            return #(false, ("Missing wheel cluster " + (#("FL", "FR", "RL", "RR"))[wheelIndex] + "."), undefined, undefined, #())

    local rootName = "VRIG_" + vehicleName + "_Root"
    if getNodeByName rootName != undefined do
        return #(false, (rootName + " already exists. Remove the generated rig before assembling again."), undefined, undefined, #())

    local root = point name:rootName size:25 box:off cross:on axistripod:on centermarker:off wirecolor:(color 120 220 255)
    root.position = vrigBoundsCenter sourceNodes
    local bodyCarrier = point name:("VRIG_" + vehicleName + "_BodyCarrier") size:20 box:off cross:on axistripod:on centermarker:off wirecolor:(color 160 255 160)
    bodyCarrier.position = root.position
    bodyCarrier.parent = root
    local wheelNames = #("VRIG_" + vehicleName + "_Wheel_FL", "VRIG_" + vehicleName + "_Wheel_FR", "VRIG_" + vehicleName + "_Wheel_RL", "VRIG_" + vehicleName + "_Wheel_RR")
    local wheels = for wheelName in wheelNames collect (point name:wheelName size:16 box:off cross:on axistripod:on centermarker:off wirecolor:(color 255 200 80))
    for wheelIndex = 1 to 4 do
    (
        local wheel = wheels[wheelIndex]
        wheel.position = vrigBoundsCenter wheelSets[wheelIndex]
        wheel.parent = root
    )

    for node in sourceNodes do
    (
        vrigRecordOriginalState node
        local wheelIndex = vrigClassifyWheelNode node
        if wheelIndex == 0 then node.parent = bodyCarrier else node.parent = wheels[wheelIndex]
    )
    #(true, ("Assembled " + (sourceNodes.count as string) + " mesh nodes non-destructively."), root, bodyCarrier, wheels)
)

fn vrigBuildOpenSpline splineName points =
(
    local path = SplineShape name:splineName
    addNewSpline path
    for point in points do addKnot path 1 #corner #line point
    updateShape path
    path
)

fn vrigAddTestLabel labelName labelText location colourValue =
(
    local label = text name:labelName text:labelText size:5.0
    label.position = location
    label.wirecolor = colourValue
    label.renderable = false
    label
)

-- Creates fixtures once and never replaces artist-authored or prior evidence.
fn vrigCreateTestPaths =
(
    local fixtureNames = #( "VRIG_Test_Simple", "VRIG_Test_Complex", "VRIG_Test_UpHill", "VRIG_Test_DownHill", "VRIG_Test_InvalidKnot" )
    for fixtureName in fixtureNames do
        if getNodeByName fixtureName != undefined do
            return #(false, (fixtureName + " already exists; keeping the existing test evidence."), #())

    local paths = #()
    append paths (vrigBuildOpenSpline "VRIG_Test_Simple" #([0, 0, 0], [30, 0, 0], [60, 12, 0], [90, 12, 0]))
    append paths (vrigBuildOpenSpline "VRIG_Test_Complex" #([0, 30, 0], [18, 30, 0], [36, 48, 0], [54, 12, 0], [72, 48, 0], [96, 30, 0]))
    append paths (vrigBuildOpenSpline "VRIG_Test_UpHill" #([0, 60, 0], [30, 60, 5], [60, 72, 12], [90, 72, 20]))
    append paths (vrigBuildOpenSpline "VRIG_Test_DownHill" #([0, 90, 20], [30, 90, 12], [60, 102, 5], [90, 102, 0]))
    append paths (vrigBuildOpenSpline "VRIG_Test_InvalidKnot" #([0, 120, 0], [30, 120, 0], [30, 120, 0], [60, 132, 0]))

    vrigAddTestLabel "VRIG_Test_Label_Simple" "VALID: simple" [0, -8, 0] (color 80 255 120)
    vrigAddTestLabel "VRIG_Test_Label_Complex" "VALID: complex" [0, 22, 0] (color 80 255 120)
    vrigAddTestLabel "VRIG_Test_Label_UpHill" "VALID: uphill" [0, 52, 0] (color 80 255 120)
    vrigAddTestLabel "VRIG_Test_Label_DownHill" "VALID: downhill" [0, 82, 20] (color 80 255 120)
    vrigAddTestLabel "VRIG_Test_Label_InvalidKnot" "EXPECTED REJECT: duplicate knot" [0, 112, 0] (color 255 90 90)
    #(true, "Created five labelled spline test fixtures.", paths)
)

-- Baked keys go on layer 1; layer 2 is zeroed and left free for manual
-- animation, so a user can offset the result without destroying the
-- generated motion. Replacing the controller stack would otherwise discard
-- the helper's rest pose, so it is captured first and returned to the
-- caller to bake on top of.
fn vrigLayeredControllers node =
(
    local restPos = in coordsys parent node.pos
    local restRot = in coordsys parent node.rotation

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
    rotationList.setName 1 "Generated Path"
    rotationList.setName 2 "Manual Offset"
    rotationList[1].controller.value = restRot
    rotationList[2].controller.value = (quat 0 0 0 1)

    positionList.setActive 2
    rotationList.setActive 2

    #(positionList[1].controller, rotationList[1].controller, restPos, restRot)
)

-- Helpers are placed on the nodes they will drive, so the vehicle keeps its
-- original layout once the source nodes are linked underneath them.
fn vrigCreateHelpers profile rootNode bodyNode wheelNodes scale =
(
    local wheelRadius = profile.tyreRadius * scale

    local master = Point name:"VRIG_Master" size:(profile.wheelbase * scale) \
        box:on cross:on axistripod:off centermarker:off wirecolor:(color 255 200 0)
    local body = Point name:"VRIG_Body" size:(profile.track * scale * 0.6) \
        box:on cross:off axistripod:off centermarker:off wirecolor:(color 120 200 255)
    local steering = Circle name:"VRIG_Steering" radius:(profile.track * scale * 0.35) \
        wirecolor:(color 255 120 200)
    local pathGuide = Point name:"VRIG_PathGuide" size:(profile.wheelbase * scale * 0.5) \
        box:off cross:on axistripod:on centermarker:off wirecolor:(color 160 255 160)

    steering.renderable = false
    master.transform = rootNode.transform

    body.parent = master
    steering.parent = master
    pathGuide.parent = master
    body.transform = bodyNode.transform
    steering.transform = bodyNode.transform
    pathGuide.transform = rootNode.transform

    local wheels = #()
    local wheelNames = #("VRIG_Wheel_FL", "VRIG_Wheel_FR", "VRIG_Wheel_RL", "VRIG_Wheel_RR")
    for index = 1 to 4 do
    (
        local wheel = Circle name:wheelNames[index] radius:wheelRadius \
            wirecolor:(color 255 255 255)
        wheel.renderable = false
        wheel.parent = master
        wheel.transform = wheelNodes[index].transform
        append wheels wheel
    )

    #(master, body, steering, pathGuide, wheels)
)

/* ------------------------------------------------------------------ *
 * Baking
 * ------------------------------------------------------------------ */

fn vrigBakeAnimation profile splineNode rootNode bodyNode wheelNodes extraNodes \
    startFrame endFrame reversed targetSpeed \
    rollIntensity accelIntensity brakeIntensity suspIntensity sampleCount \
    scale forwardOffsetDeg =
(
    for index = 1 to 4 do
        if wheelNodes[index] == undefined do return #(false, "All four wheels must be assigned", 1.0)
    if bodyNode == undefined do return #(false, "The body must be assigned", 1.0)
    if scale <= 0.0 do return #(false, "Scene units per metre must be greater than zero", 1.0)
    local points = vrigSamplePath splineNode sampleCount scale
    if reversed do
    (
        local flipped = #()
        for index = points.count to 1 by -1 do append flipped points[index]
        points = flipped
    )

    local distances = vrigCumulativeDistances points
    local curvatures = vrigCurvatures points
    local speeds = vrigSpeedProfile points distances curvatures profile targetSpeed
    local times = vrigTravelTimes distances speeds
    local naturalDuration = times[times.count]

    local frameSpan = (endFrame - startFrame) as float
    if frameSpan <= 0.0 do return #(false, "End frame must be greater than start frame", 1.0)
    animationRange = interval startFrame endFrame
    local availableSeconds = frameSpan / (frameRate as float)
    if availableSeconds <= 0.0 do return #(false, "Frame range is too short to bake", 1.0)
    if naturalDuration <= 0.0 do return #(false, "Path produced no usable motion", 1.0)

    -- Fit the physical profile to the requested frame range, keeping the
    -- relative speed variation intact.
    local timeScale = naturalDuration / availableSeconds

    local helpers = vrigCreateHelpers profile rootNode bodyNode wheelNodes scale
    local master = helpers[1]
    local body = helpers[2]
    local steering = helpers[3]
    local pathGuide = helpers[4]
    local wheels = helpers[5]

    local masterControllers = vrigLayeredControllers master
    local bodyControllers = vrigLayeredControllers body
    local guideControllers = vrigLayeredControllers pathGuide
    local wheelControllers = for wheel in wheels collect (vrigLayeredControllers wheel)

    local wheelSpin = #(0.0, 0.0, 0.0, 0.0)
    local previousDistance = 0.0
    local previousSpeed = 0.0

    with animate on
    (
        for frame = startFrame to endFrame do
        (
            local elapsed = ((frame - startFrame) as float) / (frameRate as float) * timeScale
            local state = vrigSampleAtTime times distances speeds curvatures points elapsed
            local travelled = state[1]
            local speed = state[2]
            local curvature = state[3]

            local worldPoint = (vrigPointAtDistance points distances travelled) * scale
            local yaw = vrigHeadingAtDistance points distances travelled

            local stepSeconds = 1.0 / (frameRate as float) * timeScale
            local longitudinal = if stepSeconds <= 1e-9 then 0.0 else ((speed - previousSpeed) / stepSeconds)
            local lateral = curvature * (vrigSquare speed)

            -- atan already returns degrees in MaxScript.
            local steerCentre = atan (profile.wheelbase * curvature)
            local steerAngles = vrigAckermannAngles steerCentre profile.wheelbase profile.track profile.maxSteer
            local response = vrigBodyResponse profile longitudinal lateral \
                rollIntensity accelIntensity brakeIntensity suspIntensity

            at time frame
            (
                -- The master drives the whole rig along the path in world
                -- space, so it takes the sampled position and heading
                -- directly. forwardOffsetDeg corrects for a vehicle that
                -- was not modelled facing +X.
                masterControllers[1].value = worldPoint
                masterControllers[2].value = (eulerAngles 0 0 (yaw + forwardOffsetDeg)) as quat

                -- Everything below is an offset from the helper's captured
                -- rest pose, so the vehicle keeps its original layout.
                bodyControllers[1].value = bodyControllers[3] + [0, 0, response[3] * scale]
                bodyControllers[2].value = \
                    ((eulerAngles response[1] response[2] 0) as quat) * bodyControllers[4]

                guideControllers[1].value = guideControllers[3]
                guideControllers[2].value = guideControllers[4]

                -- Which front wheel is the inner one swaps with the turn
                -- direction: positive steer is a left turn.
                local steerLeft = if steerCentre >= 0.0 then steerAngles[1] else steerAngles[2]
                local steerRight = if steerCentre >= 0.0 then steerAngles[2] else steerAngles[1]

                local travelStep = travelled - previousDistance
                for index = 1 to wheels.count do
                (
                    wheelSpin[index] += vrigWheelRotationDegrees travelStep profile.tyreRadius
                    local steer = case index of
                    (
                        1: steerLeft
                        2: steerRight
                        default: 0.0
                    )
                    wheelControllers[index][1].value = wheelControllers[index][3]
                    wheelControllers[index][2].value = \
                        ((eulerAngles 0 (-wheelSpin[index]) steer) as quat) * wheelControllers[index][4]
                )
            )

            previousDistance = travelled
            previousSpeed = speed
        )
    )

    -- Bind the vehicle only after a successful bake. Every node records its
    -- original parent and transform first, so removal restores it exactly.
    vrigRecordOriginalState rootNode
    rootNode.parent = master

    if bodyNode != rootNode do
    (
        vrigRecordOriginalState bodyNode
        bodyNode.parent = body
    )

    for index = 1 to 4 do
    (
        if wheelNodes[index] != rootNode do
        (
            vrigRecordOriginalState wheelNodes[index]
            wheelNodes[index].parent = wheels[index]
        )
    )

    for extra in extraNodes where extra != undefined and extra != rootNode do
    (
        vrigRecordOriginalState extra
        extra.parent = steering
    )

    local warning = ""
    if timeScale > 1.0 do
        warning = "Frame range is shorter than the physical drive time; motion was sped up by " \
            + (formattedPrint timeScale format:".2f") + "x."
    #(true, warning, timeScale)
)

/* ------------------------------------------------------------------ *
 * Dialogue
 * ------------------------------------------------------------------ */

rollout VehicleRigRollout "Create Vehicle Rig" width:360
(
    local boundRoot = undefined
    local boundBody = undefined
    local boundSteeringWheel = undefined
    local boundWheels = #(undefined, undefined, undefined, undefined)
    local boundSpline = undefined
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
        dropdownlist presetList "Vehicle preset" items:#("car") width:160
        spinner tyreRadiusSpin "Tyre radius (m)" range:[0.05, 1.5, 0.31] type:#float scale:0.01
        -- Defaults to the scene unit setting, but a model built at a
        -- different scale needs the real figure: measure the wheelbase in
        -- scene units and divide by the preset wheelbase in metres.
        spinner unitScaleSpin "Scene units per metre" range:[1e-6, 1e6, 1.0] type:#float scale:0.1
        dropdownlist forwardAxisList "Vehicle faces" items:#("+X", "-X", "+Y", "-Y") width:160
    )

    group "Path and Motion"
    (
        pickbutton splinePick "Pick Open Spline" width:160 across:2 message:"Pick one open spline"
        label splineLabel "none" align:#left
        spinner startFrameSpin "Start frame" range:[-100000, 100000, 0] type:#integer
        spinner endFrameSpin "End frame" range:[-100000, 100000, 100] type:#integer
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

    edittext reportBox "" height:120 width:340 readonly:true

    fn activeProfile =
    (
        local profile = vrigPresetByName (presetList.selected)
        if profile == undefined then undefined
        else
        (
            local copy = vrigCopyProfile profile
            copy.tyreRadius = tyreRadiusSpin.value
            copy
        )
    )

    -- Yaw correction so the modelled forward axis ends up along the path.
    fn forwardOffsetDegrees =
    (
        case forwardAxisList.selection of
        (
            1: 0.0
            2: 180.0
            3: -90.0
            4: 90.0
            default: 0.0
        )
    )

    fn report lines =
    (
        local text = ""
        for line in lines do text += line + "\r\n"
        reportBox.text = text
    )

    fn invalidateBake =
    (
        preflightPassed = false
        acknowledgedWarnings = false
        acknowledgeCheck.checked = false
        acknowledgeCheck.enabled = false
        bakeButton.enabled = false
    )

    fn bindingErrors =
    (
        local errors = #()
        if boundRoot == undefined do append errors "Pick the vehicle root"
        if boundBody == undefined do append errors "Pick the body"
        for index = 1 to 4 do
            if boundWheels[index] == undefined do
                append errors ("Pick wheel " + (#("FL", "FR", "RL", "RR"))[index])
        if boundRoot != undefined do
        (
            local assigned = #(boundBody, boundWheels[1], boundWheels[2], boundWheels[3], boundWheels[4])
            for node in assigned where node != undefined do
                if not (vrigIsDescendantOf node boundRoot) do
                    append errors (node.name + " is outside the selected vehicle hierarchy")
        )
        if tyreRadiusSpin.value <= 0.0 do append errors "Tyre radius must be greater than zero"
        errors
    )

    on VehicleRigRollout open do
    (
        presetList.items = vrigPresetNames()
        presetList.selection = 1
        startFrameSpin.value = animationRange.start.frame as integer
        endFrameSpin.value = animationRange.end.frame as integer
        local profile = vrigPresetByName (presetList.selected)
        if profile != undefined do tyreRadiusSpin.value = profile.tyreRadius
        unitScaleSpin.value = vrigUnitScale()
        forwardAxisList.selection = 1
        report #("Pick the vehicle root, body, four wheels, and one open spline, then run Preflight.")
    )

    on presetList selected index do
    (
        local profile = vrigPresetByName (presetList.selected)
        if profile != undefined do tyreRadiusSpin.value = profile.tyreRadius
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
        local result = vrigAssembleSelectedVehicle vehicleName
        if result[1] then
        (
            boundRoot = result[3]
            boundBody = result[4]
            boundWheels = result[5]
            rootLabel.text = boundRoot.name
            bodyLabel.text = boundBody.name
            wheelFLLabel.text = boundWheels[1].name
            wheelFRLabel.text = boundWheels[2].name
            wheelRLLabel.text = boundWheels[3].name
            wheelRRLabel.text = boundWheels[4].name
            report #(result[2], "Vehicle Root and all wheel clusters are bound. Pick an open spline, then run Preflight.")
            invalidateBake()
        )
        else report #("Assembly failed:", result[2])
    )

    on createTestsButton pressed do
    (
        local result = vrigCreateTestPaths()
        if result[1] then report #(result[2], "Simple, Complex, UpHill and DownHill are VALID. InvalidKnot is EXPECTED REJECT.")
        else report #("Test-path creation skipped:", result[2])
    )

    on rootPick picked node do ( boundRoot = node; rootLabel.text = node.name; invalidateBake() )
    on bodyPick picked node do ( boundBody = node; bodyLabel.text = node.name; invalidateBake() )
    on wheelFLPick picked node do ( boundWheels[1] = node; wheelFLLabel.text = node.name; invalidateBake() )
    on wheelFRPick picked node do ( boundWheels[2] = node; wheelFRLabel.text = node.name; invalidateBake() )
    on wheelRLPick picked node do ( boundWheels[3] = node; wheelRLLabel.text = node.name; invalidateBake() )
    on wheelRRPick picked node do ( boundWheels[4] = node; wheelRRLabel.text = node.name; invalidateBake() )
    on splinePick picked node do ( boundSpline = node; splineLabel.text = node.name; invalidateBake() )

    on tyreRadiusSpin changed value do invalidateBake()
    on unitScaleSpin changed value do invalidateBake()
    on forwardAxisList selected index do invalidateBake()
    on reverseCheck changed state do invalidateBake()
    on startFrameSpin changed value do invalidateBake()
    on endFrameSpin changed value do invalidateBake()
    on sampleSpin changed value do invalidateBake()

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

        if errors.count > 0 then
        (
            append lines "Preflight failed:"
            join lines errors
            report lines
        )
        else
        (
            local points = vrigSamplePath boundSpline sampleSpin.value unitScaleSpin.value
            local issues = vrigPreflightOpenPath points profile
            local errorCount = vrigCountBySeverity issues "error"
            local warningCount = vrigCountBySeverity issues "warning"

            if errorCount > 0 then
            (
                append lines ("Preflight failed with " + (errorCount as string) + " error(s):")
                join lines (vrigFormatIssues issues)
            )
            else if warningCount > 0 then
            (
                preflightPassed = true
                acknowledgeCheck.enabled = true
                append lines ("Preflight passed with " + (warningCount as string) + " warning(s).")
                append lines "Review each warning, then tick Accept warnings to enable baking."
                join lines (vrigFormatIssues issues)
            )
            else
            (
                preflightPassed = true
                acknowledgeCheck.enabled = true
                acknowledgeCheck.checked = true
                acknowledgedWarnings = true
                bakeButton.enabled = true
                append lines "Preflight passed with no issues."
                append lines ("Path length: " + (formattedPrint (curveLength boundSpline 1) format:".2f") + " system units.")
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
            local profile = activeProfile()
            local extras = #()
            if boundSteeringWheel != undefined do append extras boundSteeringWheel

            undo "Bake Vehicle Rig" on
            (
                local result = vrigBakeAnimation profile boundSpline boundRoot boundBody \
                    boundWheels extras \
                    startFrameSpin.value endFrameSpin.value reverseCheck.checked \
                    targetSpeedSpin.value rollSpin.value accelSpin.value brakeSpin.value \
                    suspensionSpin.value sampleSpin.value unitScaleSpin.value \
                    (forwardOffsetDegrees())

                if result[1] then
                (
                    local lines = #("Bake complete.")
                    append lines ("Frames " + (startFrameSpin.value as string) + " to " + (endFrameSpin.value as string) + ".")
                    if result[2] != "" do append lines result[2]
                    append lines "Manual Offset is the active controller layer; edit it without losing the baked path."
                    report lines
                )
                else report #("Bake failed:", result[2])
            )
            invalidateBake()
        )
    )

    on removeButton pressed do
    (
        undo "Remove Vehicle Rig" on
        (
            local result = vrigRemoveGeneratedRig()
            report #(
                "Removed " + (result[2] as string) + " generated node(s).",
                "Restored " + (result[1] as string) + " bound node(s) to their original parent and transform."
            )
        )
        invalidateBake()
    )
)

if VehicleRigFloater != undefined and VehicleRigFloater != false do
    closeRolloutFloater VehicleRigFloater
global VehicleRigFloater = newRolloutFloater "Vehicle Rig" 380 760
addRollout VehicleRigRollout VehicleRigFloater
