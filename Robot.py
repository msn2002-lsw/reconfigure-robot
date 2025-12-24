import Leg
import Gait
import Tool
import Config
import time

class robot:
    def __init__(self, model):

        self.stepSizeStand = 80   # 机器人步长，单位mm
        self.stepSizeCrawl = 80
        self.heightStand = 30          # 抬腿高度，单位mm
        self.heightCrawl = 30          # 抬腿高度，单位mm

        self.stepTimeStand = 2.0  # 机器人步态周期，单位s
        self.stepTimeCrawl = 2.0

        self.state = "crawl"  # 机器人当前状态：站立式/匍匐式  

        self.triangularGaitInfo = [
        Config.LegParam(name='LF', offset=0.0, duty_factor=0.75),
        Config.LegParam(name='RF', offset=0.5, duty_factor=0.75),
        Config.LegParam(name='LB', offset=0.75, duty_factor=0.75),
        Config.LegParam(name='RB', offset=0.25, duty_factor=0.75)
        ]

    def change_step_size_stand(self, newStepSize):
        if newStepSize >= 0:
            self.stepSizeStand = newStepSize
        pass

    def change_step_size_crawl(self, newStepSize):
        if newStepSize >= 0:
            self.stepSizeCrawl = newStepSize
        pass

    def change_step_time_stand(self, newStepTime):
        if newStepTime >= 0:
            self.stepTimeStand = newStepTime
        pass

    def change_step_time_crawl(self, newStepTime):
        if newStepTime >= 0:
            self.stepTimeCrawl = newStepTime
        pass

    def move_triangular_gait(self):
        time_start = time.time()
        t_total = self.stepTimeCrawl
        # 初始化各条腿相位
        tPhase = {}
        tPhaseNow = {}
        for leg in self.triangularGaitInfo:
            tPhase[leg.name] = 0

        while time.time() - time_start < t_total:
            time_now = time.time() - time_start
            positions = Gait.getPositionInGaitCycle(self.triangularGaitInfo, time_now, t_total, self.heightCrawl, self.stepSizeCrawl)
            # 计算此时相位
            for leg in self.triangularGaitInfo:
                tPhaseNow[leg.name] = Gait.isSwing(leg.offset, leg.duty_factor, time_now, t_total)[0]
                if tPhase[leg.name] > tPhaseNow[leg.name]:
                    # 更新腿的初始位置
                    pass

            time.sleep(0.1)
        pass

    def move_diagonal_gait(self):
        pass

    def tansition_to_stand(self):
        self.state = 'stand'
        pass

    def tansition_to_crawl(self):
        self.state = 'crawl'
        pass