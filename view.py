import pygame
import random


class Robot:
    __robot_image = pygame.image.load('pic/robot.png')

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_image(self):
        return self.__robot_image


robot = Robot(-1, -1)

butter_image = pygame.image.load('pic/Butter.png')
target_image = pygame.image.load('pic/target.png')

obstacles = [pygame.image.load('pic/bread.png'), pygame.image.load('pic/coffee.png'),
             pygame.image.load('pic/breakfast.png'), pygame.image.load('pic/juice.png'),
             pygame.image.load('pic/milk.png')]

obstacles_state = []


def update_state(state, move):
    global robot

    new_state = state.copy()
    new_state[robot.x, robot.y] = new_state[robot.x, robot.y][:-1]

    if move == 'U':
        if 'b' in state[robot.x - 1, robot.y]:

            new_state[robot.x - 1, robot.y] = new_state[robot.x - 1, robot.y][:-1]

            if 'p' in state[robot.x - 2, robot.y]:
                new_state[robot.x - 2, robot.y] = new_state[robot.x - 2, robot.y][:-1] + 'b'
            else:
                new_state[robot.x - 2, robot.y] += 'b'

        new_state[robot.x - 1, robot.y] += 'r'
        robot.x = robot.x - 1

    elif move == 'D':
        if 'b' in state[robot.x + 1, robot.y]:

            new_state[robot.x + 1, robot.y] = new_state[robot.x + 1, robot.y][:-1]

            if 'p' in state[robot.x + 2, robot.y]:
                new_state[robot.x + 2, robot.y] = new_state[robot.x + 2, robot.y][:-1] + 'b'
            else:
                new_state[robot.x + 2, robot.y] += 'b'

        new_state[robot.x + 1, robot.y] += 'r'
        robot.x = robot.x + 1

    elif move == 'L':
        if 'b' in state[robot.x, robot.y - 1]:

            new_state[robot.x, robot.y - 1] = new_state[robot.x, robot.y - 1][:-1]

            if 'p' in state[robot.x, robot.y - 2]:
                new_state[robot.x, robot.y - 2] = new_state[robot.x, robot.y - 2][:-1] + 'b'
            else:
                new_state[robot.x, robot.y - 2] += 'b'

        new_state[robot.x, robot.y - 1] += 'r'
        robot.y = robot.y - 1

    else:
        if 'b' in state[robot.x, robot.y + 1]:

            new_state[robot.x, robot.y + 1] = new_state[robot.x, robot.y + 1][:-1]

            if 'p' in state[robot.x, robot.y + 2]:
                new_state[robot.x, robot.y + 2] = new_state[robot.x, robot.y + 2][:-1] + 'b'
            else:
                new_state[robot.x, robot.y + 2] += 'b'

        new_state[robot.x, robot.y + 1] += 'r'
        robot.y = robot.y + 1

    return new_state


def redraw_window(window, state):
    window.fill((89, 17, 17))

    col_counter = 120
    row_counter = 120

    obstacles_counter = 0

    for i in range(window.get_size()[1] // 120 - 1):
        pygame.draw.line(window, (148, 101, 52), (0, col_counter), (window.get_size()[0], col_counter), width=1)
        col_counter += 120

    for j in range(window.get_size()[0] // 120 - 1):
        pygame.draw.line(window, (148, 101, 52), (row_counter, 0), (row_counter, window.get_size()[1]), width=1)
        row_counter += 120

    for i in range(state.shape[0]):
        for j in range(state.shape[1]):

            if 'x' in state[i][j]:
                obstacle_png = pygame.transform.scale(obstacles_state[obstacles_counter], (120, 120))
                obstacles_counter += 1
                window.blit(obstacle_png, (j * 120, i * 120))

            elif 'r' in state[i][j]:
                robot_png = pygame.transform.scale(robot.get_image(), (120, 110))
                window.blit(robot_png, (j * 120, i * 120 + 5))

            elif 'b' in state[i][j]:
                butter_png = pygame.transform.scale(butter_image, (120, 120))
                window.blit(butter_png, (j * 120, i * 120))

            elif 'p' in state[i][j]:
                target_png = pygame.transform.scale(target_image, (160, 120))
                window.blit(target_png, (j * 120 - 20, i * 120))

    pygame.display.update()


def start(init_state, moves):
    global obstacles_state
    obstacles_state = random.sample(obstacles * 50, init_state.shape[0] * init_state.shape[1])

    global robot
    for i in range(init_state.shape[0]):
        for j in range(init_state.shape[1]):
            if 'r' in init_state[i, j]:
                robot.x = i
                robot.y = j
                break

    pygame.init()
    clock = pygame.time.Clock()

    window = pygame.display.set_mode((120 * init_state.shape[1], 120 * init_state.shape[0]))

    pygame.display.set_caption("What's my purpose ???")
    pygame.display.set_icon(robot.get_image())
    state = init_state
    while len(moves) > 0:
        clock.tick(64)

        state = update_state(state, moves.pop(0))
        pygame.time.delay(750)
        redraw_window(window, state)

        for event1 in pygame.event.get():
            if event1.type == pygame.QUIT:
                pygame.quit()
                exit(0)

    pygame.time.delay(2500)
    pygame.quit()
