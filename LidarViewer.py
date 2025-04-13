import os
import time
import pygame
import subprocess
import multiprocessing
import select
from math import pi, sin, cos

FIFO_PATH = "/tmp/lidarpipe"
LIDAR_READER_EXEC = "./lidar_reader"

class LidarViewer:
    def __init__(self, fifo_path=FIFO_PATH, ws=640):
        self.fifo_path = fifo_path
        self.ws = ws
        self.hws = ws // 2
        self.center = (self.hws, self.hws)

        self.scale = 25.0
        self.max_scale = 8000 / self.hws
        self.dot_size = 3
        self.gridstyle = 0
        self.running = True

        self.set_dark_mode()

        pygame.init()
        self.screen = pygame.display.set_mode((ws, ws))
        pygame.display.set_caption("Lidar Viewer")
        pygame.font.init()
        self.font = pygame.font.SysFont('Calibri', 16)
        self.clock = pygame.time.Clock()

        if not os.path.exists(self.fifo_path):
            os.mkfifo(self.fifo_path)

        self.reader_proc = subprocess.Popen([LIDAR_READER_EXEC])
        os.sched_setaffinity(self.reader_proc.pid, {1})
#        print(f"LidarReader iniciado no PID {self.reader_proc.pid} (núcleo 1)")

        self.fifo_fd = os.open(self.fifo_path, os.O_RDONLY | os.O_NONBLOCK)
        self.fifo_file = os.fdopen(self.fifo_fd)
        self.datapack = []
        self.last_delay_ms = -1
        self.grid_surface = self.create_grid()

    def set_dark_mode(self):
        self.backcolor = (0, 0, 0)
        self.gridcolor = (80, 80, 255)
        self.lessergridcolor = (0, 0, 96)
        self.gridcentercolor = (255, 255, 255)

    def set_light_mode(self):
        self.backcolor = (255, 255, 255)
        self.gridcolor = (0, 0, 128)
        self.lessergridcolor = (192, 192, 255)
        self.gridcentercolor = (0, 0, 0)

    def create_grid(self):
        surface = pygame.Surface((self.ws, self.ws)).convert()
        surface.fill(self.backcolor)
        major = self.hws // 8
        minor = max(1, major // 5)

        if self.gridstyle == 0:
            for i in range(8):
                pygame.draw.circle(surface, self.gridcolor, self.center, (i + 1) * major, 1)
                for j in range(minor, major, minor):
                    pygame.draw.circle(surface, self.lessergridcolor, self.center, (i + 1) * major - j, 1)
            for i in range(0, 360, 90):
                for j in range(15, 90, 15):
                    a = pi * (i + j) / 180
                    x = self.hws + self.hws * sin(a)
                    y = self.hws + self.hws * cos(a)
                    pygame.draw.line(surface, self.lessergridcolor, self.center, (x, y), 1)
        else:
            for i in range(0, self.ws + 1, minor):
                pygame.draw.line(surface, self.lessergridcolor, (0, i), (self.ws, i))
                pygame.draw.line(surface, self.lessergridcolor, (i, 0), (i, self.ws))
            for i in range(0, self.ws + 1, major):
                pygame.draw.line(surface, self.gridcolor, (0, i), (self.ws, i))
                pygame.draw.line(surface, self.gridcolor, (i, 0), (i, self.ws))

        pygame.draw.line(surface, self.gridcentercolor, (0, self.hws), (self.ws, self.hws))
        pygame.draw.line(surface, self.gridcentercolor, (self.hws, 0), (self.hws, self.ws))

        for i in range(8):
            dist = (i + 1) * 1000 * (self.scale / self.max_scale)
            txt = self.font.render(f"{int(dist)} mm", False, self.gridcolor)
            surface.blit(txt, (self.hws + 10, self.hws - ((i + 1) * major)))
            surface.blit(txt, (self.hws + 10, self.hws + ((i + 1) * major)))

        return surface

    def polar_to_point(self, polar):
        angle, dist = polar
        ang = angle * pi / 180.0
        dist /= self.scale
        return (self.center[0] - dist * sin(ang), self.center[1] + dist * cos(ang))

    def update_screen(self):
        self.screen.fill(self.backcolor)
        self.screen.blit(self.grid_surface, (0, 0))
        for polar in self.datapack:
            pt = self.polar_to_point(polar)
            pygame.draw.circle(self.screen, (255, 0, 0), (int(pt[0]), int(pt[1])), self.dot_size)
        if self.last_delay_ms >= 0:
            text = self.font.render(f"Delay: {self.last_delay_ms} ms", False, (255, 255, 0))
            self.screen.blit(text, (10, self.ws - 20))
        pygame.display.update()

    def handle_newframe(self, line):
 #       print(f"[Viewer] {time.time()*1000:.0f} ms - NEWFRAME linha recebida: {line}")
        try:
            parts = line.split()
            if len(parts) == 2:
                frame_ts = int(parts[1])
                now = int(time.time() * 1000)
                self.last_delay_ms = now - frame_ts
                if self.last_delay_ms > 1000:
  #                  print("Delay elevado, limpando backlog")
                    os.read(self.fifo_fd, 4096)
            else:
                self.last_delay_ms = -1
        except:
            self.last_delay_ms = -1

    def handle_key(self, key):
        if key in [pygame.K_KP_PLUS, pygame.K_EQUALS] and self.scale > 3:
            self.scale /= 2
        elif key in [pygame.K_KP_MINUS, pygame.K_MINUS] and self.scale < self.max_scale:
            self.scale *= 2
        elif key in [pygame.K_1, pygame.K_KP1]: self.scale = self.max_scale / 8
        elif key in [pygame.K_2, pygame.K_KP2]: self.scale = self.max_scale / 4
        elif key in [pygame.K_3, pygame.K_KP3]: self.scale = self.max_scale * 3 / 8
        elif key in [pygame.K_4, pygame.K_KP4]: self.scale = self.max_scale / 2
        elif key in [pygame.K_5, pygame.K_KP5]: self.scale = self.max_scale * 5 / 8
        elif key in [pygame.K_6, pygame.K_KP6]: self.scale = self.max_scale * 3 / 4
        elif key in [pygame.K_7, pygame.K_KP7]: self.scale = self.max_scale * 7 / 8
        elif key in [pygame.K_8, pygame.K_KP8]: self.scale = self.max_scale
        elif key == pygame.K_LEFTBRACKET and self.dot_size > 1:
            self.dot_size -= 1
        elif key == pygame.K_RIGHTBRACKET and self.dot_size < 6:
            self.dot_size += 1
        elif key == pygame.K_g:
            self.gridstyle = (self.gridstyle + 1) % 2
        elif key == pygame.K_d:
            self.set_dark_mode()
        elif key == pygame.K_b:
            self.set_light_mode()
        else:
            return
        self.grid_surface = self.create_grid()

    def run(self):
        frame_count = 0
        while self.running:
            frame_line, frame_points = self.skip_to_last_frame()
            if frame_line:
                self.datapack = frame_points
                self.handle_newframe(frame_line)
                frame_count += 1
   #             print(f"[Viewer] Frame {frame_count} exibido com delay {self.last_delay_ms} ms")
                self.update_screen()
            
                
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and event.key in [pygame.K_ESCAPE, pygame.K_q]):
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_f:
                        self.save_frame_to_csv()
                    else:
                        self.handle_key(event.key)

            self.clock.tick(60)

        self.reader_proc.terminate()
        self.reader_proc.wait()

    def save_frame_to_csv(self):
        timestamp = int(time.time() * 1000)
        filename = f"frame_{timestamp}.csv"
        with open(filename, 'w') as f:
            f.write("angle,distance\n")
            for angle, dist in self.datapack:
                f.write(f"{angle:.2f},{dist}\n")
        print(f"Frame salvo em {filename}")

    def skip_to_last_frame(self):
        latest_frame = None
        latest_points = []
        while True:
            rlist, _, _ = select.select([self.fifo_fd], [], [], 0.01)
            if not rlist:
                break
            line = self.fifo_file.readline().strip()
            if line.startswith("NEWFRAME"):
                latest_frame = line
            else:
                try:
                    a, d = map(float, line.split(','))
                    latest_points.append((a, d))
                except:
                    continue
        return latest_frame, latest_points

if __name__ == '__main__':
    os.sched_setaffinity(0, {2})  # Viewer no núcleo 2
    viewer = LidarViewer()
    viewer.run()
