 
from bpy import context, data, ops
import random, os
import math
import json
from mathutils import Matrix, Vector

def RandFloat(L, R) :
    return L + random.random() * (R - L)
    
def GenerateLights() :
    n = random.randint(1, 3)
    for _ in range(n) :
        while True :
            position = (RandFloat(-6.0, 6.0), RandFloat(-6.0, 6.0), RandFloat(-6.0, 6.0))
            length = 0
            for p in position :
                length += p * p
            if length > 16.0 :
                break
        ops.object.lamp_add(
            type = 'POINT',
            view_align = False,
            location = position
        )
        
def GenerateRandomCurve(curve_name = 'MyCurve') :
    # Create a bezier circle and enter edit mode.
    # if random.randint(0, 1) == 1 :
    ops.curve.primitive_bezier_curve_add(radius=random.random(),
                                        location=(0.0, 0.0, 0.0),
                                        enter_editmode=True)
    #else :
    #    ops.curve.primitive_bezier_circle_add(radius=random.random(),
    #                                         location=(0.0, 0.0, 0.0),
    #                                         enter_editmode=True)

    # Subdivide the curve by a number of cuts, giving the
    # random vertex function more points to work with.
    ops.curve.subdivide(number_cuts = 2)

    # Randomize the vertices of the bezier circle.
    # offset [-inf .. inf], uniform [0.0 .. 1.0],
    # normal [0.0 .. 1.0], RNG seed [0 .. 10000].
    ops.transform.vertex_random(offset=1.0, uniform=0.1, normal=0.0, seed=0)

    # Scale the curve while in edit mode.
    ops.transform.resize(
        value = (RandFloat(0.5, 3.0), RandFloat(0.5, 3.0), RandFloat(0.5, 3.0))
    )
    #ops.transform.translate(
    #    value = (RandFloat(-4.0, 4.0), RandFloat(-4.0, 4.0), RandFloat(-4.0, 4.0)))
    ops.transform.rotate(
        value = RandFloat(0, math.pi),
        axis = (RandFloat(-1.0, 1.0), RandFloat(-1.0, 1.0), RandFloat(-1.0, 1.0))
    )
    # Return to object mode.
    ops.object.mode_set(mode='OBJECT')

    # Store a shortcut to the curve object's data.
    context.active_object.name = context.active_object.data.name = curve_name
    obj_data = context.active_object.data
    # Which parts of the curve to extrude ['HALF', 'FRONT', 'BACK', 'FULL'].
    obj_data.fill_mode = 'FULL'

    # Breadth of extrusion.
    obj_data.extrude = 0

    # Smoothness of the segments on the curve.
    obj_data.resolution_u = 20
    obj_data.render_resolution_u = 32

    # ops.curve.primitive_bezier_circle_add(radius=random.random() * 0.03 + 0.02, enter_editmode=True)
    ops.curve.primitive_bezier_circle_add(radius=0.005, enter_editmode=True)
    ops.curve.subdivide(number_cuts = 1)
    bevel_control = context.active_object
    bevel_control.data.name = bevel_control.name = 'Bevel Control'

    obj_data.bevel_object = bevel_control
    ops.object.mode_set(mode='OBJECT')
    
    mat = obj_data.materials.get("Material")
    if mat == None:
        # create material
        mat = data.materials.new(name="Material")

    # Assign it to object
    if obj_data.materials:
        # assign to 1st material slot
        obj_data.materials[0] = mat
    else:
        # no slots
        obj_data.materials.append(mat)
    # mat.diffuse_color = (random.random(), random.random(), random.random())
    mat.diffuse_color = (0.0, 0.0, 0.0)

def ClearAllObjects() :
    ops.object.select_all(action='DESELECT')
    for obj in data.objects :
        obj.select = True
    ops.object.delete()

def GenerateRenderResult(camera_name = 'camera', img_name = 'tmp') :
    # context.scene.render.engine = 'CYCLES'
    context.scene.render.engine = 'BLENDER_RENDER'
    # context.scene.cycles.film_transparent = False
    for obj in data.objects:
        if obj.type == 'CAMERA' and obj.name == camera_name:
            context.scene.camera = obj
            print('Set camera %s' % obj.name)
            file_name = '/home/aska/Data/' + img_name
            context.scene.render.filepath = file_name + '.jpg'
            ops.render.render(write_still=True)

def GenerateManyCurveImages(num_img) :
    l = len(str(num_img))
    for _ in range(num_img) :
        ClearAllObjects()
        GenerateRandomCurve()
        # GenerateLights()
        GenerateRenderResult('camera', ('%.' + str(l) + 'd') % _)

def GenerateCameras(num_camera, is_plane = False) :
    if not is_plane : 
        n = int(math.sqrt(num_camera))
        for i in range(n) :
            for j in range(n) :
                a = 2.0 * math.pi / n * i
                b = math.pi * (-0.5) + (math.pi / n * (j + 0.5))
                x = 10 * math.cos(a) * math.cos(b)
                y = 10 * math.sin(a) * math.cos(b)
                z = 10 * math.sin(b)   
                ops.object.camera_add(
                    view_align=True,
                    enter_editmode=False,
                    location=(x, y, z)
                )
                context.active_object.name = 'Camera.' + str(i * n + j)
                context.active_object.data.name = 'Camera.' + str(i * n + j) 
    else :
        n = num_camera
        for i in range(n) :
            a = 2.0 * math.pi / n * i
            x = 10 * math.cos(a)
            y = 10 * math.sin(a)
            z = 0.0
            ops.object.camera_add(
                view_align=True,
                enter_editmode=False,
                location=(x, y, z)
            )
            context.active_object.name = 'Camera.' + str(i)
            context.active_object.data.name = 'Camera.' + str(i)
            

def MakeCamerasLookAt(curve_name = 'MyCurve') :
    for ob in context.scene.objects :
        if ob.type != 'CAMERA' :
            continue
        context.scene.objects.active = ob
        ops.object.constraint_add(type='TRACK_TO')
        context.object.constraints["Track To"].target = data.objects[curve_name]
        context.object.constraints["Track To"].track_axis = 'TRACK_NEGATIVE_Z'
        context.object.constraints["Track To"].up_axis = 'UP_Y'
        
def GenerateCurveImagesWithManyCameras(num_camera, is_plane = False) :
    ClearAllObjects()
    GenerateRandomCurve('MyCurve')
    GenerateCameras(num_camera, is_plane)
    MakeCamerasLookAt('MyCurve')
    # GenerateLights()
    for ob in context.scene.objects :
        if ob.type != 'CAMERA' :
            continue
        GenerateRenderResult(ob.name, ob.name[7:])

#---------------------------------------------------------------------------------------------------
# 3x4 P matrix from Blender camera
# Reference: https://blender.stackexchange.com/questions/38009/3x4-camera-matrix-from-blender-camera
#---------------------------------------------------------------------------------------------------

# BKE_camera_sensor_size
def GetSenserSize(sensor_fit, sensor_x, sensor_y):
    if sensor_fit == 'VERTICAL':
        return sensor_y
    return sensor_x

# BKE_camera_sensor_fit
def GetSensorFit(sensor_fit, size_x, size_y):
    if sensor_fit == 'AUTO':
        if size_x >= size_y:
            return 'HORIZONTAL'
        else:
            return 'VERTICAL'
    return sensor_fit

# Build intrinsic camera parameters from Blender camera data
#
# See notes on this in 
# blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model
# as well as
# https://blender.stackexchange.com/a/120063/3581
def GetCalibrationMatrixKFromBlender(camd):
    if camd.type != 'PERSP':
        raise ValueError('Non-perspective cameras not supported')
    scene = context.scene
    f_in_mm = camd.lens
    scale = scene.render.resolution_percentage / 100
    resolution_x_in_px = scale * scene.render.resolution_x
    resolution_y_in_px = scale * scene.render.resolution_y
    sensor_size_in_mm = GetSenserSize(camd.sensor_fit, camd.sensor_width, camd.sensor_height)
    sensor_fit = GetSensorFit(
        camd.sensor_fit,
        scene.render.pixel_aspect_x * resolution_x_in_px,
        scene.render.pixel_aspect_y * resolution_y_in_px
    )
    pixel_aspect_ratio = scene.render.pixel_aspect_y / scene.render.pixel_aspect_x
    if sensor_fit == 'HORIZONTAL':
        view_fac_in_px = resolution_x_in_px
    else:
        view_fac_in_px = pixel_aspect_ratio * resolution_y_in_px
    pixel_size_mm_per_px = sensor_size_in_mm / f_in_mm / view_fac_in_px
    s_u = 1 / pixel_size_mm_per_px
    s_v = 1 / pixel_size_mm_per_px / pixel_aspect_ratio

    # Parameters of intrinsic calibration matrix K
    u_0 = resolution_x_in_px / 2 - camd.shift_x * view_fac_in_px
    v_0 = resolution_y_in_px / 2 + camd.shift_y * view_fac_in_px / pixel_aspect_ratio
    skew = 0 # only use rectangular pixels

    K = Matrix(
        ((s_u, skew, u_0),
        (   0,  s_v, v_0),
        (   0,    0,   1)))
    return K

# Returns camera rotation and translation matrices from Blender.
# 
# There are 3 coordinate systems involved:
#    1. The World coordinates: "world"
#       - right-handed
#    2. The Blender camera coordinates: "bcam"
#       - x is horizontal
#       - y is up
#       - right-handed: negative z look-at direction
#    3. The desired computer vision camera coordinates: "cv"
#       - x is horizontal
#       - y is down (to align to the actual pixel coordinates 
#         used in digital images)
#       - right-handed: positive z look-at direction

def Get3x4RTMatrixFromBlender(cam):
    # bcam stands for blender camera
    R_bcam2cv = Matrix(
        ((1, 0,  0),
        (0, -1, 0),
        (0, 0, -1)))

    # Transpose since the rotation is object rotation, 
    # and we want coordinate rotation
    # R_world2bcam = cam.rotation_euler.to_matrix().transposed()
    # T_world2bcam = -1*R_world2bcam * location
    #
    # Use matrix_world instead to account for all constraints
    location, rotation = cam.matrix_world.decompose()[0:2]
    print('location = ' + str(location))
    R_world2bcam = rotation.to_matrix().transposed()

    # Convert camera location to translation vector used in coordinate changes
    # T_world2bcam = -1*R_world2bcam*cam.location
    # Use location from matrix_world to account for constraints:     
    T_world2bcam = -1*R_world2bcam * location
    print('T_world2bcam = ' + str(T_world2bcam))
    # Build the coordinate transform matrix from world to computer vision camera
    R_world2cv = R_bcam2cv*R_world2bcam
    T_world2cv = R_bcam2cv*T_world2bcam

    # put into 3x4 matrix
    RT = Matrix((
        R_world2cv[0][:] + (T_world2cv[0],),
        R_world2cv[1][:] + (T_world2cv[1],),
        R_world2cv[2][:] + (T_world2cv[2],)
        ))
    return R_world2cv, T_world2cv, RT

def Get3x4PMatrixFromBlender(cam, add_error = False):
    K = GetCalibrationMatrixKFromBlender(cam.data)
    R, T, RT = Get3x4RTMatrixFromBlender(cam)
    if add_error :
        for i in range(3) :
            T[i] += RandFloat(-0.05, 0.05)
    return K, R, T

def GetCameraMatrixFromBlender(add_error = False) :
    f = open('/home/aska/Data/Cameras.txt', 'w')
    text = ''
    for cam in context.scene.objects :
        if cam.type != 'CAMERA' :
            continue
        print(cam.name)
        K, R, T = Get3x4PMatrixFromBlender(cam, add_error)
        for i in range(3) :
            for j in range(3) :
                text += str(K[i][j]) + ' '
            text += '\n'
        for i in range(3) :
            for j in range(3) :
                text += str(R[i][j]) + ' '
            text += '\n'
        for i in range(3) :
            text += str(T[i]) + '\n'
    f.write(text)
    f.close()

def GeneratePointCloud() :
    file_path = '/home/totoro/CG/ReproduceThem/ReconstructThin/Testing/points.json'
    f = open(file_path, 'r')
    points = json.loads(f.read())['points']
    f.close()
    for pt in points :
        ops.mesh.primitive_cube_add(location = tuple(pt), enter_editmode = True)
        ops.transform.resize(value=(0.002, 0.002, 0.002))

GenerateCurveImagesWithManyCameras(64, is_plane=True)
GetCameraMatrixFromBlender(add_error = True)
