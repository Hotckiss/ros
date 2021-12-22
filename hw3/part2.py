from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


# Solution's id	0a8fb730-2bfa-4730-986f-06e9e1b1dad1
# User	hotckiss_e.moevm.info
# Task's id	duckietown1
# Task's loc_id	duckietown_1
# Datetime	2021-12-22 20:54:03.121000
# You can obtain logs here: https://github.com/OSLL/aido-auto-feedback/tree/5787c46b98e67d0ecff38c5d85025a0c2cbff0cc715a4f9d212d78c3

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
        print(f'start_cnt={start_cnt}')
        while condition:
            img, _, _, _ = env.step([1, 0])

            cnt = self.count(img)
            print(f'before={cnt}')
            if 4.0 * start_cnt < cnt:
                env.step([0, 45])
                for i in range(8):
                    env.step([1, 0])
                img, _, _, _ = env.step([0, -45])

                cnt = self.count(img)
                print(f'after={cnt}')
                while 2.0 * start_cnt < cnt:
                    img, _, _, _ = env.step([1, 0])
                    cnt = self.count(img)
                    print(f'after={cnt}')

                env.step([0, -45])
                for i in range(8):
                    env.step([1, 0])
                env.step([0, 45])

                condition = False

            env.render(img)