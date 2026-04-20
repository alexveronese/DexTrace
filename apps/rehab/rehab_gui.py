import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import random
import pygame
import time
import math
import sys

TREMOR_THRESH = 0.5

class RehabApp(Node):
    def __init__(self):
        super().__init__('rehab_app')
        self.sub = self.create_subscription(Twist, '/surgeon_input', self.pos_callback, 10)
        self.pub = self.create_publisher(Bool, '/collision_alert', 10)
        
        pygame.init()
        
        # --- MODIFICA 1: SCHERMO INTERO DINAMICO ---
        # --- DIMENSIONI FISSE PER STABILITÀ SU WSL2 ---
        self.w, self.h = 1280, 720
        self.screen = pygame.display.set_mode((self.w, self.h))
        pygame.display.set_caption("Virtual Rehab - WSL2 Stable Version")
        
        self.center_x, self.center_y = self.w // 2, self.h // 2
        
        self.user_pos = [float(self.center_x), float(self.center_y)]
        self.target_pos = [float(self.center_x), float(self.center_y)]

        self.angle = 0.0
        self.active = False
        self.last_active_state = False

        self.error_threshold = 45.0 
        self.speed = 0.008     
        self.tremor_val = 0.0      

    def pos_callback(self, msg):
        new_active = msg.angular.z > 0.5
        
        # --- NOVITÀ: SNAP AL TARGET ALLA PARTENZA ---
        if new_active and not self.last_active_state:
            self.user_pos = [float(self.center_x), float(self.center_y)]
            self.angle = 0.0
            print("Exercise started: cursor initialized!")

        self.active = new_active
        self.last_active_state = new_active

        if self.active:
            self.user_pos[0] += msg.linear.x * 10
            self.user_pos[1] += msg.linear.y * 10

        self.tremor_val = msg.angular.x


    def run(self):
        clock = pygame.time.Clock()
        try:
            while rclpy.ok():
                for event in pygame.event.get():
                    # ESC per uscire dal fullscreen
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        return
                    if event.type == pygame.QUIT:
                        return

                self.screen.fill((15, 15, 15))

                if not self.active:
                    self.target_pos = [self.center_x, self.center_y]
                    font = pygame.font.SysFont(None, 72)
                    text = font.render("PRESS THE BUTTON TO START", True, (0, 200, 255))
                    text_rect = text.get_rect(center=(self.center_x, self.center_y - 100))
                    self.screen.blit(text, text_rect)
                    
                    # Disegna target di attesa
                    pygame.draw.circle(self.screen, (100, 100, 100), self.target_pos, int(self.error_threshold), 2)
                    
                    # Forza allarme spento mentre non è attivo
                    msg = Bool()
                    msg.data = False
                    self.pub.publish(msg)
                else:
                    # --- MODIFICA 2: PARTENZA DAL CENTRO ---
                    # Usiamo una funzione a raggio crescente per far uscire il target dal centro
                    # oppure semplicemente iniziamo l'orbita. Qui facciamo un'orbita ellittica:
                    expansion = (1 - math.exp(-self.angle))
                    self.target_pos[0] = self.center_x + (400 * expansion) * math.cos(self.angle)
                    self.target_pos[1] = self.center_y + (250 * expansion) * math.sin(self.angle)
                    self.angle += self.speed

                    # Calcolo Distanza ed Errore
                    dist = math.sqrt((self.user_pos[0]-self.target_pos[0])**2 + (self.user_pos[1]-self.target_pos[1])**2)
                    
                    color = (0, 255, 0) 
                    is_alarm = False

                    if dist > self.error_threshold:
                        color = (255, 0, 0)
                        is_alarm = True
                    elif dist > self.error_threshold * 0.75:
                        color = (255, 255, 0) # GIALLO sul bordo

                    user_color = (255, 255, 255)
                    shake_offset = [0, 0]
                    if self.tremor_val > TREMOR_THRESH: # Soglia tremore
                        user_color = (255, 165, 0) # Arancione
                        shake_offset = [random.uniform(-2, 2), random.uniform(-2, 2)]

                    alert_msg = Bool()
                    alert_msg.data = is_alarm
                    self.pub.publish(alert_msg)

                    pygame.draw.circle(self.screen, color, (int(self.target_pos[0]), int(self.target_pos[1])), int(self.error_threshold), 4)
                    pygame.draw.circle(self.screen, user_color, (int(self.user_pos[0] + shake_offset[0]), int(self.user_pos[1] + shake_offset[1])), 10)

                    # HUD Info
                    font_small = pygame.font.SysFont(None, 30)
                    tremor_text = font_small.render(f"Tremor: {self.tremor_val:.2f}", True, user_color)
                    self.screen.blit(tremor_text, (20, 20))
                
                pygame.display.flip()
                clock.tick(60)
                rclpy.spin_once(self, timeout_sec=0.001)
        finally:
            print("Chiusura in corso: reset allarmi...")
            msg = Bool()
            msg.data = False
            for _ in range(10): # Aumentiamo a 10 per sicurezza
                self.pub.publish(msg)
            
            # Fondamentale: diamo tempo a ROS di inviare i pacchetti
            time.sleep(0.2) 
            
            pygame.quit()


def main():
    rclpy.init()
    node = RehabApp()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()