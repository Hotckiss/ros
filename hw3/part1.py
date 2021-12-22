from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def count(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        return np.array(cv2.inRange(hsv, np.array([10, 100, 100], dtype='uint8'), np.array([40, 255, 255], dtype='uint8')), dtype=np.bool).sum()

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        condition = True
        start_cnt = self.count(img)
        while condition:
            img, reward, done, info = env.step([1, 0])
            # img in RGB
            # add here some image processing
            env.render()
            if 4.0 * start_cnt < self.count(img):
                env.step([0, 0])
                env.render()
                condition = False


# Solution's id	502a95d6-cd50-4e7c-b6c0-94487ad724eb
# User	hotckiss_e.moevm.info
# Task's id	duckietown1
# Task's loc_id	duckietown_1
# Datetime	2021-12-22 19:44:01.642000
# You can obtain logs here: https://github.com/OSLL/aido-auto-feedback/tree/e1c0c6f2280289d0c710ebe3c5d3358b2f30880275edbace5ac78b33