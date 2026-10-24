import carla
import random
import queue
import numpy as np
import pygame
import pandas as pd

# ================== CẤU HÌNH ==================
WIN_W, WIN_H = 1280, 720
DT = 0.05                 # fixed_delta_seconds (20 Hz)
NUM_STEPS = 1000          # đặt None để chạy đến khi đóng cửa sổ / nhấn ESC
TM_PORT = 8000            # cổng Traffic Manager (điều khiển autopilot)

# Các góc nhìn camera: (tên, transform tương đối so với xe)
CAMERA_VIEWS = [
    ("Driver POV", carla.Transform(carla.Location(x=0.5, y=0.0, z=1.4))),
    ("Chase cam",  carla.Transform(carla.Location(x=-6.0, z=2.5), carla.Rotation(pitch=-10))),
    ("Top-down",   carla.Transform(carla.Location(x=0.0, z=20.0), carla.Rotation(pitch=-90))),
]


# ================== HÀM PHỤ ==================
def spawn_camera(world, bp_library, ego_car, view_idx):
    """Spawn camera RGB gắn vào xe, trả về (camera, queue ảnh)."""
    bp = bp_library.find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(WIN_W))
    bp.set_attribute('image_size_y', str(WIN_H))
    bp.set_attribute('fov', '90')
    cam = world.spawn_actor(bp, CAMERA_VIEWS[view_idx][1], attach_to=ego_car)
    q = queue.Queue()
    cam.listen(q.put)  # callback chạy ở thread khác -> đẩy vào queue
    return cam, q


def image_to_surface(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]  # BGRA -> RGB
    return pygame.surfarray.make_surface(arr.swapaxes(0, 1))


def draw_hud(screen, font, lines):
    panel = pygame.Surface((330, 14 + 22 * len(lines)), pygame.SRCALPHA)
    panel.fill((0, 0, 0, 140))
    screen.blit(panel, (10, 10))
    for i, text in enumerate(lines):
        screen.blit(font.render(text, True, (255, 255, 255)), (22, 18 + 22 * i))


def manual_control(keys, control):
    """WASD + SPACE, có làm mượt góc lái giống manual_control.py."""
    control.throttle = 1.0 if keys[pygame.K_w] else 0.0
    control.brake = 1.0 if keys[pygame.K_s] else 0.0
    control.hand_brake = bool(keys[pygame.K_SPACE])
    if keys[pygame.K_a]:
        control.steer = max(-0.7, control.steer - 0.05) if control.steer <= 0 else 0.0
    elif keys[pygame.K_d]:
        control.steer = min(0.7, control.steer + 0.05) if control.steer >= 0 else 0.0
    else:
        control.steer = 0.0


# ================== KHỞI TẠO CARLA ==================
client = carla.Client('localhost', 2000)
client.set_timeout(60.0)
world = client.get_world()
carla_map = world.get_map()

bp_library = world.get_blueprint_library()
transportation_bp = bp_library.filter('vehicle.*')
spawn_points = carla_map.get_spawn_points()
ego_car = world.try_spawn_actor(random.choice(transportation_bp), random.choice(spawn_points))
if ego_car is None:
    raise RuntimeError("Spawn xe thất bại (điểm spawn bị chiếm), hãy chạy lại")

# Spectator của cửa sổ Unreal (nếu có) vẫn bám theo xe
spectator = world.get_spectator()
t = ego_car.get_transform()
spectator.set_transform(carla.Transform(
    t.location - t.get_forward_vector() * 8 + carla.Location(z=3),
    carla.Rotation(pitch=-15, yaw=t.rotation.yaw)))

camera = None
tm = None
telemetry_data = []
pygame.init()
pygame.font.init()

try:
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DT
    world.apply_settings(settings)

    # Traffic Manager cũng phải chạy synchronous khi world ở sync mode
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)
    # Tuỳ chọn tinh chỉnh autopilot của xe ego:
    # tm.vehicle_percentage_speed_difference(ego_car, 0)   # % chậm hơn tốc độ giới hạn (âm = nhanh hơn)
    # tm.distance_to_leading_vehicle(ego_car, 3.0)         # khoảng cách an toàn (m)
    # tm.ignore_lights_percentage(ego_car, 0)              # % khả năng vượt đèn đỏ
    # tm.auto_lane_change(ego_car, True)

    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA Ego View")
    font = pygame.font.SysFont("consolas,dejavusansmono,monospace", 16)
    clock = pygame.time.Clock()

    view_idx = 0
    camera, image_queue = spawn_camera(world, bp_library, ego_car, view_idx)
    last_surface = None

    manual_mode = False
    ego_car.set_autopilot(True, TM_PORT)       # bật lái tự động ngay từ đầu
    control = carla.VehicleControl()

    step = 0
    running = True
    while running and (NUM_STEPS is None or step < NUM_STEPS):
        # ---- Sự kiện cửa sổ ----
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_TAB:      # đổi góc camera
                    camera.stop()
                    camera.destroy()
                    view_idx = (view_idx + 1) % len(CAMERA_VIEWS)
                    camera, image_queue = spawn_camera(world, bp_library, ego_car, view_idx)
                elif event.key == pygame.K_m:        # đổi autopilot <-> lái tay
                    manual_mode = not manual_mode
                    control = carla.VehicleControl()
                    ego_car.set_autopilot(not manual_mode, TM_PORT)

        # ---- Điều khiển ----
        if manual_mode:
            manual_control(pygame.key.get_pressed(), control)
            ego_car.apply_control(control)
        # Autopilot: Traffic Manager tự điều khiển, KHÔNG gọi apply_control

        # ---- Tick đồng bộ ----
        world.tick()

        if not manual_mode:
            control = ego_car.get_control()  # đọc lại lệnh autopilot vừa áp dụng để ghi log/HUD

        # ---- Lấy ảnh camera và vẽ ----
        try:
            last_surface = image_to_surface(image_queue.get(timeout=1.0))
        except queue.Empty:
            pass  # camera vừa respawn, dùng lại khung trước
        if last_surface is not None:
            screen.blit(last_surface, (0, 0))

        # ---- Đọc trạng thái xe ----
        transform = ego_car.get_transform()
        velocity = ego_car.get_velocity()
        accel = ego_car.get_acceleration()

        x, y = transform.location.x, transform.location.y
        pitch, yaw, roll = (transform.rotation.pitch, transform.rotation.yaw,
                            transform.rotation.roll)
        vx, vy = velocity.x, velocity.y
        v = (vx**2 + vy**2) ** 0.5
        ax, ay = accel.x, accel.y

        telemetry_data.append({
            'time_step': step, 'x': x, 'y': y, 'v': v, 'vx': vx, 'vy': vy,
            'ax': ax, 'ay': ay, 'pitch': pitch, 'yaw': yaw, 'roll': roll,
            'steer': control.steer, 'throttle': control.throttle, 'brake': control.brake
        })

        # ---- HUD ----
        draw_hud(screen, font, [
            f"Vehicle: {ego_car.type_id}",
            f"Map:     {carla_map.name.split('/')[-1]}",
            f"Step:    {step}   Time: {step * DT:6.1f} s",
            f"FPS:     {clock.get_fps():5.1f}",
            f"Speed:   {3.6 * v:5.1f} km/h",
            f"Pos:     ({x:7.1f}, {y:7.1f})",
            f"Yaw:     {yaw:6.1f} deg",
            f"Accel:   ({ax:5.2f}, {ay:5.2f}) m/s2",
            f"Throttle {control.throttle:4.2f}  Brake {control.brake:4.2f}",
            f"Steer    {control.steer:5.2f}",
            f"Mode:    {'MANUAL (WASD)' if manual_mode else 'AUTOPILOT'}",
            f"Camera:  {CAMERA_VIEWS[view_idx][0]}",
            "TAB camera | M mode | ESC quit",
        ])
        pygame.display.flip()

        clock.tick(int(1 / DT))  # chạy đúng thời gian thực (20 FPS)
        step += 1

finally:
    if telemetry_data:
        pd.DataFrame(telemetry_data).to_csv("testrun_output.csv", index=False)
    if camera is not None:
        camera.stop()
        camera.destroy()
    pygame.quit()

    ego_car.set_autopilot(False, TM_PORT)
    if tm is not None:
        tm.set_synchronous_mode(False)

    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    print("Đã trả về Asynchronous mode thành công")
