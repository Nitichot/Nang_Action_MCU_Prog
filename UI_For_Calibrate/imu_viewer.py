import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import threading
import sys

# ==========================================
# 1. ตั้งค่า Serial Port 
# ==========================================
COM_PORT = 'COM16'  # เปลี่ยนให้ตรงกับบอร์ดของคุณ
BAUD_RATE = 115200

# เก็บค่าที่แสดงผลบนหน้าจอ
sensor_data = {
    'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
    'pos_x': 0.0, 'pos_y': 0.0, 'pos_z': 0.0
}

raw_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}

tuning_params = {
    'beta': 0.05,
    'deadzone': 0.025,
    'damping': 0.98
}

def read_serial_data():
    global sensor_data, raw_pos
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {COM_PORT}")
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                try:
                    values = list(map(float, line.split(',')))
                    if len(values) == 6:
                        sensor_data['pitch'] = values[0]
                        sensor_data['roll'] = values[1]
                        sensor_data['yaw'] = values[2]
                        
                        raw_pos['x'] = values[3]
                        raw_pos['y'] = values[4]
                        raw_pos['z'] = values[5]
                        
                        scale = 10.0
                        sensor_data['pos_x'] = (raw_pos['x'] - offset['x']) * scale
                        sensor_data['pos_y'] = (raw_pos['y'] - offset['y']) * scale
                        sensor_data['pos_z'] = (raw_pos['z'] - offset['z']) * scale
                except ValueError:
                    pass 
    except Exception as e:
        print(f"Serial Error: {e}")
        sys.exit()

def draw_pcb_board():
    vertices = (
        ( 1.5, -0.1, -1), ( 1.5,  0.1, -1),
        (-1.5,  0.1, -1), (-1.5, -0.1, -1),
        ( 1.5, -0.1,  1), ( 1.5,  0.1,  1),
        (-1.5, -0.1,  1), (-1.5,  0.1,  1)
    )
    edges = ((0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7))
    surfaces = ((0,1,2,3), (3,2,6,7), (7,6,5,4), (4,5,1,0), (1,5,6,2), (4,0,3,7))
    
    glBegin(GL_QUADS)
    for surface in surfaces:
        x = 0
        for vertex in surface:
            x += 1
            glColor3fv((0.0, 0.4, 0.7) if x % 2 == 0 else (0.1, 0.5, 0.8))
            glVertex3fv(vertices[vertex])
    glEnd()

    glBegin(GL_LINES)
    glColor3fv((0,0,0))
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def draw_text(surface, font, text, x, y, color=(255, 255, 255)):
    text_surface = font.render(text, True, color)
    surface.blit(text_surface, (x, y))

# ==========================================
# ฟังก์ชันแปลง 2D UI ให้แสดงบน 3D OpenGL
# ==========================================
def draw_2d_overlay(surface, width, height):
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0, width, height, 0)
    
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()
    
    glDisable(GL_DEPTH_TEST) 
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_TEXTURE_2D)
    
    texture_data = pygame.image.tostring(surface, "RGBA", False)
    texture_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_data)
    
    # รีเซ็ตสีของ OpenGL ให้เป็นสีขาว 100% ก่อนแปะ Texture UI
    glColor3f(1.0, 1.0, 1.0)
    
    glBegin(GL_QUADS)
    glTexCoord2f(0, 0); glVertex2f(0, 0)
    glTexCoord2f(1, 0); glVertex2f(width, 0)
    glTexCoord2f(1, 1); glVertex2f(width, height)
    glTexCoord2f(0, 1); glVertex2f(0, height)
    glEnd()
    
    glDeleteTextures(1, [texture_id])
    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST) 
    
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()

def main():
    global offset, raw_pos, tuning_params
    
    pygame.init()
    display = (1000, 600)
    screen = pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("IMU 3D Visualizer & Tuner")
    
    font_large = pygame.font.SysFont('Consolas', 22, bold=True)
    font_medium = pygame.font.SysFont('Consolas', 18)
    font_small = pygame.font.SysFont('Consolas', 14)

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(2.0, 0.0, -12) 
    glEnable(GL_DEPTH_TEST)

    serial_thread = threading.Thread(target=read_serial_data, daemon=True)
    serial_thread.start()

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r or event.key == pygame.K_SPACE:
                    offset['x'] = raw_pos['x']
                    offset['y'] = raw_pos['y']
                    offset['z'] = raw_pos['z']
                    print("Position Reset!")
                
                elif event.key == pygame.K_1: tuning_params['beta'] = round(tuning_params['beta'] + 0.01, 3)
                elif event.key == pygame.K_2: tuning_params['beta'] = round(max(0, tuning_params['beta'] - 0.01), 3)
                elif event.key == pygame.K_3: tuning_params['deadzone'] = round(tuning_params['deadzone'] + 0.005, 3)
                elif event.key == pygame.K_4: tuning_params['deadzone'] = round(max(0, tuning_params['deadzone'] - 0.005), 3)

        glClearColor(0.15, 0.15, 0.15, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # วาด 3D
        glPushMatrix()
        glTranslatef(sensor_data['pos_x'], sensor_data['pos_y'], sensor_data['pos_z'])
        glRotatef(sensor_data['yaw'], 0, 1, 0)
        glRotatef(sensor_data['pitch'], -1, 0, 0)
        glRotatef(sensor_data['roll'], 0, 0, -1)
        draw_pcb_board()
        glPopMatrix()

        # วาด 2D UI
        info_surface = pygame.Surface(display, pygame.SRCALPHA)
        
        # กล่องข้อมูลเซ็นเซอร์
        pygame.draw.rect(info_surface, (20, 20, 30, 220), (10, 10, 300, 260), border_radius=8)
        pygame.draw.rect(info_surface, (100, 150, 255, 200), (10, 10, 300, 260), width=2, border_radius=8)
        draw_text(info_surface, font_large, "SENSOR DATA", 20, 20, (255, 220, 0))
        pygame.draw.line(info_surface, (100, 150, 255), (20, 45), (290, 45), 2)
        
        draw_text(info_surface, font_medium, "ORIENTATION", 20, 60, (150, 200, 255))
        draw_text(info_surface, font_medium, f"Pitch : {sensor_data['pitch']:7.2f} °", 30, 85)
        draw_text(info_surface, font_medium, f"Roll  : {sensor_data['roll']:7.2f} °", 30, 110)
        draw_text(info_surface, font_medium, f"Yaw   : {sensor_data['yaw']:7.2f} °", 30, 135)
        
        draw_text(info_surface, font_medium, "POSITION (XYZ)", 20, 170, (150, 255, 150))
        draw_text(info_surface, font_medium, f"Pos X : {sensor_data['pos_x']:7.2f} cm", 30, 195, (200, 255, 200))
        draw_text(info_surface, font_medium, f"Pos Y : {sensor_data['pos_y']:7.2f} cm", 30, 220, (200, 255, 200))
        draw_text(info_surface, font_medium, f"Pos Z : {sensor_data['pos_z']:7.2f} cm", 30, 245, (200, 255, 200))

        # กล่อง Tuning
        pygame.draw.rect(info_surface, (30, 20, 20, 220), (10, 290, 300, 180), border_radius=8)
        pygame.draw.rect(info_surface, (255, 150, 100, 200), (10, 290, 300, 180), width=2, border_radius=8)
        draw_text(info_surface, font_large, "TUNING PARAMS", 20, 300, (255, 150, 50))
        pygame.draw.line(info_surface, (255, 150, 100), (20, 325), (290, 325), 2)
        
        draw_text(info_surface, font_small, f"[1]/[2] Beta     : {tuning_params['beta']:.3f}", 20, 340)
        draw_text(info_surface, font_small, f"[3]/[4] Deadzone : {tuning_params['deadzone']:.3f} g", 20, 365)
        draw_text(info_surface, font_small, f"        Damping  : {tuning_params['damping']:.2f}", 20, 390)
        draw_text(info_surface, font_small, "* Note: Edit code in ESP32 to apply", 20, 430, (150,150,150))

        # ปุ่ม Reset
        pygame.draw.rect(info_surface, (40, 40, 40, 200), (display[0]-320, display[1]-60, 310, 50), border_radius=8)
        draw_text(info_surface, font_medium, "Press 'SPACE' to Reset Pos", display[0]-300, display[1]-45, (255, 255, 255))

        draw_2d_overlay(info_surface, display[0], display[1])
        
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()