import numpy as np
import pygame

global G

G = 6.67430e-11  # Gravitational constant

# Initialize Pygame
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

# Circle Class
class Circle:
    def __init__(self, x, y, mass, radius):
        self.position = np.array([x, y], dtype='float64')
        self.mass = mass
        self.radius = radius
        self.velocity = np.array([0, 0], dtype='float64')
        self.trail = []  # List to store previous positions
    
    def wrap_around(self, screen_width, screen_height):
        if self.position[0] < 0:  # If off screen to the left
            self.position[0] = screen_width
        elif self.position[0] > screen_width:  # If off screen to the right
            self.position[0] = 0
        
        if self.position[1] < 0:  # If off screen to the top
            self.position[1] = screen_height
        elif self.position[1] > screen_height:  # If off screen to the bottom
            self.position[1] = 0

    def draw(self):
        pygame.draw.circle(screen, (255, 255, 255), self.position.astype(int), self.radius)
        
        # Draw the trail
        if len(self.trail) > 1:
            pygame.draw.lines(screen, (255, 0, 0), False, [pos.astype(int) for pos in self.trail], 2)

    def update(self, other):
        r_vec = other.position - self.position
        r_mag = np.linalg.norm(r_vec)

        # Gravity calculation
        r_hat = r_vec / r_mag
        force = G * self.mass * other.mass / r_mag**2
        self.velocity += r_hat * force / self.mass

        # Check for collision
        if r_mag < self.radius + other.radius:
            # Correct for overlap
            overlap = 0.5 * (r_mag - self.radius - other.radius)
            self.position -= overlap * (self.position - other.position) / r_mag
            other.position += overlap * (self.position - other.position) / r_mag

            # Recalculate distance after overlap correction
            r_vec = other.position - self.position
            r_mag = np.linalg.norm(r_vec)
            normal = r_vec / r_mag
            tangent = np.array([-normal[1], normal[0]])

            # Decompose velocities
            v1n = np.dot(normal, self.velocity)
            v1t = np.dot(tangent, self.velocity)

            v2n = np.dot(normal, other.velocity)
            v2t = np.dot(tangent, other.velocity)

            # Elastic collision for normal components
            m1, m2 = self.mass, other.mass
            new_v1n = (v1n * (m1 - m2) + 2 * m2 * v2n) / (m1 + m2)
            new_v2n = (v2n * (m2 - m1) + 2 * m1 * v1n) / (m1 + m2)

            # Convert scalar normal and tangential velocities into vectors
            new_v1n_v = new_v1n * normal
            v1t_v = v1t * tangent

            new_v2n_v = new_v2n * normal
            v2t_v = v2t * tangent

            # Recompose velocities
            self.velocity = new_v1n_v + v1t_v
            other.velocity = new_v2n_v + v2t_v
        
        self.trail.append(self.position.copy())
        if len(self.trail) > 50:  # Limit the trail length
            self.trail.pop(0)


# Create two circles
# Initialize circles
circle1 = Circle(400, 300, 10e15, 15)  # Central body
circle2 = Circle(200, 300, 10e13, 10)   # Satellite

# Calculate distance and orbital velocity
distance = np.linalg.norm(circle2.position - circle1.position)
orbital_velocity = np.sqrt((G * circle1.mass) / distance)

# Increase the initial velocity
velocity_factor = 7  # Adjust this factor as needed
circle2.velocity = np.array([0, -orbital_velocity * velocity_factor])

# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((0, 0, 0))  # Clear screen

    # Update and draw circles
    circle1.update(circle2)
    circle2.update(circle1)
    circle1.position += circle1.velocity * clock.get_time() / 1000.0
    circle2.position += circle2.velocity * clock.get_time() / 1000.0

    circle1.wrap_around(width, height)
    circle2.wrap_around(width, height)

    circle1.draw()
    circle2.draw()

    pygame.display.flip()  # Update screen
    clock.tick(60)  # 60 frames per second

pygame.quit()
