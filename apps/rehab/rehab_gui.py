import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pygame
import time
import math
import random
import sys

# --- Graphics and Levels ---
COLORS = {
    "bg": (10, 12, 15),
    "panel": (25, 30, 40),
    "text": (240, 240, 240),
    "accent": (0, 200, 255),
    "danger": (255, 50, 50),
    "success": (50, 255, 100),
    "warning": (255, 165, 0),
    "border": (255, 255, 0)
}

LEVELS = {
    1: {"name": "REACHING", "gain": 1.2, "err_max": 55.0, "eval_thresh": 2.5,  "speed": 0.0,   "t_thresh": 0.7},
    2: {"name": "TRACKING", "gain": 2.4, "err_max": 45.0, "eval_thresh": 35.0, "speed": 0.008, "t_thresh": 0.5},
    3: {"name": "CHASING",  "gain": 3.0, "err_max": 40.0, "eval_thresh": 50.0, "speed": 0.015, "t_thresh": 0.3}
}

class RehabApp(Node):
    def __init__(self):
        super().__init__('rehab_app')
        self.sub = self.create_subscription(Twist, '/surgeon_input', self.pos_callback, 10)
        self.pub_alert = self.create_publisher(Bool, '/collision_alert', 10)
        self.pub_config = self.create_publisher(Twist, '/rehab_config', 10)
        
        pygame.init()
        self.w, self.h = 1280, 720
        self.screen = pygame.display.set_mode((self.w, self.h))
        pygame.display.set_caption("DexTrace - v1.0")
        
        self.f_huge = pygame.font.SysFont("Verdana", 90, bold=True)
        self.f_big = pygame.font.SysFont("Verdana", 50, bold=True)
        self.f_med = pygame.font.SysFont("Verdana", 35)
        
        self.state = "WAITING" 
        self.level = 1
        self.center_x, self.center_y = self.w // 2, self.h // 2
        self.user_pos = [float(self.center_x), float(self.center_y)]
        
        self.last_transition_time = 0.0
        self.debounce_delay = 0.6
        self.pico_active = False
        self.last_pico_active = False

    def pos_callback(self, msg):
        self.pico_active = msg.angular.z > 0.5
        self.tremor_val = msg.angular.x
        now = time.monotonic()

        if self.pico_active and not self.last_pico_active:
            if (now - self.last_transition_time) > self.debounce_delay:
                if self.level == 4: 
                    self.safe_exit()
                elif self.state in ["WAITING", "RESULTS"]: self.load_level()
                elif self.state == "LOADED": self.start_exercise()
                self.last_transition_time = now
        
        elif not self.pico_active and self.last_pico_active:
            if self.state == "ACTIVE" and (now - self.last_transition_time) > self.debounce_delay:
                self.stop_exercise(interrupted=True)
                self.last_transition_time = now

        self.last_pico_active = self.pico_active
        if self.state == "ACTIVE":
            self.user_pos[0] += msg.linear.x * 10
            self.user_pos[1] += msg.linear.y * 10

    def load_level(self):
        self.state = "LOADED"
        m = Twist()
        m.linear.x = LEVELS[self.level]["gain"]
        m.linear.y = LEVELS[self.level]["t_thresh"]
        self.pub_config.publish(m)
        self.pub_alert.publish(Bool(data=False))        #RESET ALARM

    def start_exercise(self):
        self.state = "ACTIVE"
        if self.level == 2:
            self.user_pos = [float(self.center_x + 250), float(self.center_y)]
        else:
            self.user_pos = [float(self.center_x), float(self.center_y)]

        self.start_pos = list(self.user_pos)
        self.path_length = 0.0
        self.last_sample = list(self.user_pos)
        self.angle = 0.0
        self.start_time = time.monotonic()
        self.session_data = []
        self.tremor_data = []

    def stop_exercise(self, interrupted=False):
        self.state = "RESULTS"
        self.pub_alert.publish(Bool(data=False))
        l_cfg = LEVELS[self.level]
        
        if interrupted:
            self.last_results = {"status": "INTERRUPTED", "msg": "Session cancelled by the user", "color": COLORS["warning"]}
            return
        
        avg_tremor = sum(self.tremor_data) / len(self.tremor_data) if self.tremor_data else 0
        tremor_penalty = avg_tremor * 10

        if self.level == 1:
            dist_to_target = math.hypot(self.user_pos[0]-self.target_pos[0], self.user_pos[1]-self.target_pos[1])
            reached = dist_to_target < l_cfg["err_max"]
            
            opt = math.hypot(self.target_pos[0]-self.start_pos[0], self.target_pos[1]-self.start_pos[1])
            path_eff = self.path_length / opt if opt > 0 else 1.0
            val = path_eff + (avg_tremor * 0.5)
            name = "Dex-Efficiency"
            success = reached and (val < l_cfg["eval_thresh"])

            if not reached:
                sugg = "Target not reached! Keep moving."
            else:
                sugg = "Level 2 unlocked" if success else "Too much tremor or long path!"

        else:
            rmse = math.sqrt(sum([d**2 for d in self.session_data])/len(self.session_data)) if self.session_data else 999
            val = rmse + tremor_penalty
            name = "Dex-Accuracy (RMSE+T)"
            success = val < l_cfg["eval_thresh"]

        if success:
            sugg = "GREAT! TEST PASSED" if self.level == 3 else f"Level {self.level+1} unlocked"
            self.level += 1
        else:
            sugg = "Try again to became a DexMaster"

        self.last_results = {
            "status": "SUCCESS" if success else "FAIL",
            "val": val, "name": name, "sugg": sugg,
            "avg_tremor": avg_tremor,
            "color": COLORS["success"] if success else COLORS["danger"]
        }

    def safe_exit(self):
        self.pub_alert.publish(Bool(data=False))
        time.sleep(0.2) 
        pygame.quit()
        sys.exit()

    def draw_ui_elements(self, elapsed, dist, l_cfg):
        # Sfondo e Target
        self.screen.fill(COLORS["bg"])
        
        color = COLORS["success"] 
        if dist > l_cfg["err_max"]:
            color = COLORS["danger"]
        elif dist > l_cfg["err_max"] * 0.75:
            color = COLORS["border"]
        
        pygame.draw.circle(self.screen, color, (int(self.target_pos[0]), int(self.target_pos[1])), int(l_cfg["err_max"]), 3)
        
        # Cursore con shake se tremore alto
        u_col = COLORS["text"] if self.tremor_val < l_cfg["t_thresh"] else COLORS["warning"]
        shake = [random.uniform(-3,3), random.uniform(-3,3)] if u_col == COLORS["warning"] else [0,0]
        pygame.draw.circle(self.screen, u_col, (int(self.user_pos[0]+shake[0]), int(self.user_pos[1]+shake[1])), 12)
        
        # Header Info
        header = self.f_med.render(f"LEVEL {self.level} - {l_cfg['name']}", True, COLORS["accent"])
        self.screen.blit(header, (40, 25))
        # HUD Info
        font_small = pygame.font.SysFont(None, 35)
        tremor_text = font_small.render(f"Tremor Intensity: {self.tremor_val:.2f}", True, u_col)
        self.screen.blit(tremor_text, (40, 80))
        
        # Barra Tempo
        pygame.draw.rect(self.screen, COLORS["panel"], (self.center_x-250, 40, 500, 15))
        pygame.draw.rect(self.screen, COLORS["accent"], (self.center_x-250, 40, int(500*(1-elapsed/10)), 15))

    def run(self):
        clock = pygame.time.Clock()
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE): 
                    self.safe_exit()

            if self.state == "WAITING":
                self.screen.fill(COLORS["bg"])
                t = self.f_big.render("DEXTRACE SYSTEM READY", True, COLORS["text"])
                self.screen.blit(t, t.get_rect(center=(self.center_x, self.center_y)))

            elif self.state == "LOADED":
                self.screen.fill(COLORS["panel"])
                t = self.f_big.render(f"LEVEL {self.level} LOADED", True, COLORS["accent"])
                s = self.f_med.render("PRESS THE BUTTON TO START", True, COLORS["text"])
                self.screen.blit(t, t.get_rect(center=(self.center_x, self.center_y-40)))
                self.screen.blit(s, s.get_rect(center=(self.center_x, self.center_y+40)))
                self.pub_alert.publish(Bool(data=False)) #HEARTBEAT FOR THE PICO WATCHDOG

            elif self.state == "ACTIVE":
                elapsed = time.monotonic() - self.start_time
                if elapsed > 10.0: self.stop_exercise(); continue
                
                l_cfg = LEVELS[self.level]
                if self.level == 1: self.target_pos = [self.center_x + 350, self.center_y]
                elif self.level == 2:
                    self.target_pos = [self.center_x + 250*math.cos(self.angle), self.center_y + 250*math.sin(self.angle)]
                    self.angle += l_cfg["speed"]
                else:
                    exp = (1 - math.exp(-self.angle))
                    self.target_pos = [self.center_x + 400*exp*math.cos(self.angle), self.center_y + 250*exp*math.sin(self.angle)]
                    self.angle += l_cfg["speed"]

                dist = math.hypot(self.user_pos[0]-self.target_pos[0], self.user_pos[1]-self.target_pos[1])
                self.session_data.append(dist)
                self.tremor_data.append(self.tremor_val)
                self.path_length += math.hypot(self.user_pos[0]-self.last_sample[0], self.user_pos[1]-self.last_sample[1])
                self.last_sample = list(self.user_pos)
                
                self.pub_alert.publish(Bool(data=(dist > l_cfg["err_max"])))
                self.draw_ui_elements(elapsed, dist, l_cfg)

            elif self.state == "RESULTS":
                self.screen.fill(COLORS["bg"])
                res = self.last_results
                title = self.f_huge.render(res["status"], True, res["color"])
                self.screen.blit(title, title.get_rect(center=(self.center_x, 180)))
                
                if "val" in res:
                    m = self.f_big.render(f"{res['name']}: {res['val']:.2f}", True, COLORS["text"])
                    t_info = self.f_med.render(f"Avg Tremor: {res['avg_tremor']:.2f}", True, COLORS["warning"])
                    s = self.f_med.render(res["sugg"], True, COLORS["accent"])
                    self.screen.blit(m, m.get_rect(center=(self.center_x, 330)))
                    self.screen.blit(t_info, t_info.get_rect(center=(self.center_x, 400)))
                    self.screen.blit(s, s.get_rect(center=(self.center_x, 500)))
                else:
                    m = self.f_big.render(res["msg"], True, COLORS["text"])
                    self.screen.blit(m, m.get_rect(center=(self.center_x, 330)))

                if self.level == 4:
                    hint = self.f_med.render("PRESS TO EXIT", True, (80, 80, 80))
                else:
                    hint = self.f_med.render("PRESS TO CONTINUE", True, (80, 80, 80))
                
                self.screen.blit(hint, hint.get_rect(center=(self.center_x, 620)))

            pygame.display.flip()
            clock.tick(60)
            rclpy.spin_once(self, timeout_sec=0.001)

if __name__ == '__main__':
    rclpy.init()
    RehabApp().run()
    rclpy.shutdown()