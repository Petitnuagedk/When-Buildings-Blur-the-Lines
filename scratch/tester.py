import sionna.rt as rt

scene = rt.load_scene(rt.scene.floor_wall)
print(list(scene.radio_materials.keys()))

scene = rt.load_scene("scratch/scene_wifi24.xml")
print(scene.radio_materials)   # must not be empty

from sionna.rt import Camera
cam = Camera(position=[70., -20., 190.], look_at=[0., 0., 4.])
scene.render_to_file(camera=cam, filename="test_render.png")