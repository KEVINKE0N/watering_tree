import machine
from machine import Pin
import time
'''import dht'''
import utime
'''from umqtt.simple import MQTTClient'''
'''import network'''
from machine import Timer


led = Pin(2, Pin.OUT)
led.off()

# 雨水传感器信号引脚
adc_rain = machine.ADC(machine.Pin(33))
adc_rain.atten(machine.ADC.ATTN_11DB)


# 流量计信号引脚
PULSE_PIN_TREE = 23
LITERS_PER_PULSE = 1 / 1920

# 继电器信号引脚
relay = machine.Pin(15, machine.Pin.OUT)

# 流量计部分
pulses_count_tree = 0
pulse_pin_tree = machine.Pin(PULSE_PIN_TREE, machine.Pin.IN)

# 定义回调函数处理树木通道的脉冲事件
def pulse_callback_tree(pin):
    global pulses_count_tree
    pulses_count_tree += 1

pulse_pin_tree.irq(trigger=machine.Pin.IRQ_RISING, handler=pulse_callback_tree)



# 主循环变量
relay.off()
start_watering_tree = False
rain_threshold = 2000  # 调整雨水传感器的阈值
water_volume_threshold = 0.02
three_days_seconds = 10

# 网络连接


#雨水传感器检测
def rain():
    analog_value = adc_rain.read()
    return 4095 - analog_value

#树木浇水逻辑
# 状态机常量
WATERING_STATES = {
    'IDLE': 0,
    'PRECHECK': 1,
    'ACTIVE': 2,
    'EMERGENCY_STOP': 3
}

class TreeWateringSystem:
    def __init__(self):
        self.state = WATERING_STATES['IDLE']
        self.timer = 0
        self.water_used = 0.0
        self.last_rain_check = utime.ticks_ms()
        self.flow_calibration = 1.0  # 流量校准系数
        
        # 硬件定时器（精度1秒）
        self.sys_timer = Timer(-1)
        self.sys_timer.init(period=1000, mode=Timer.PERIODIC, callback=self._tick)



    # 状态机回调
    def _tick(self, t):
        global pulses_count_tree
        """每秒触发的定时器回调"""
        # 实时雨水监测（即使正在浇水）
        if utime.ticks_diff(utime.ticks_ms(), self.last_rain_check) > 500:
            if rain() >= rain_threshold:
                if self.state != WATERING_STATES['IDLE']:
                    self._emergency_stop()
            self.last_rain_check = utime.ticks_ms()
            
        # 状态机处理
        if self.state == WATERING_STATES['PRECHECK']:
            self.timer += 1
            print(f"---------------\nPrecheck {self.timer}s")
            if self.timer >= three_days_seconds:
                self._start_watering()
                
                
        if self.state == WATERING_STATES['ACTIVE']:
            # 精确流量计算（带时间补偿）
            pulses = pulses_count_tree  # 原子操作读取
            self.water_used += (pulses * LITERS_PER_PULSE) * self.flow_calibration
            pulses_count_tree = 0  # 重置计数
            
            if self.water_used >= water_volume_threshold:
                self._stop_watering(success=True)
                
    def _start_watering(self):
        """启动浇水流程"""
        relay.on()
        self.state = WATERING_STATES['ACTIVE']
        pulses_count_tree = 0  # 重置脉冲计数
        print("---------------\nWatering started")
        
    def _stop_watering(self, success=False):
        """停止浇水"""
        relay.off()
        self.state = WATERING_STATES['IDLE']
        if not success:
            print("---------------\nEmergency stopped")
        else:
            print(f"---------------\nWatered {self.water_used:.2f}L")
        self.water_used = 0.0
        self.timer = 0
        
    def _emergency_stop(self):
        """紧急停止"""
        self.state = WATERING_STATES['EMERGENCY_STOP']
        self._stop_watering()
        
    def update(self):
        """主循环调用"""
        if self.state == WATERING_STATES['IDLE']:
            if rain() < rain_threshold:
                self.state = WATERING_STATES['PRECHECK']
                print("-----------------\nstart precheck")

# 初始化系统
watering_system = TreeWateringSystem() 

# 主循环简化为
while True:
    watering_system.update()
    time.sleep(1)


##测试测试