class Filter:
    def __init__(self) -> None:
        pass
    
    def add_value(self, value):
        pass
    
    def get_value(self):
        pass
    
class WeightedAverageFilter:
    def __init__(self, window_size=10, weight=[1,2,3]):
        self.window_size = window_size
        self.data_len = 0
        self.data = []
        self.weight = weight
        self.weight_n = len(weight)
        self.value = 0

    def add_value(self, value):
        self.data.append(value)
        # 保持数据窗口不超过窗口大小
        if len(self.data) > self.window_size:
            self.data.pop(0)
        self.data_len = len(self.data)
        self.value = self.__get_average()
        
    def get_value(self):
        return self.value

    def __get_average(self):
        s = 0
        ws = 0
        for i in range(self.data_len):
            weight_i = round(self.weight_n * i / self.data_len)
            s += self.data[i] * self.weight[weight_i]
            ws += self.weight[weight_i]
        return s/ws if ws != 0 else 0
    
    
class MovingAverageFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.data = []
        self.result = 0

    def add_sample(self, new_sample):
        self.data.append(new_sample)
        # 保持数据窗口不超过窗口大小
        if len(self.data) > self.window_size:
            self.data.pop(0)
        self.result = self.__get_average()
        

    def get_value(self, new_sample = None):
        if new_sample is not None:
            self.add_sample(new_sample)
        return self.result
    
    def __get_average(self):
        return sum(self.data) / len(self.data) if len(self.data) != 0 else 0
    
    
def saturation(num, max, min):
    if num > max:
        return max
    if num < min:
        return min
    return num

def dead_zone(num, dead_up, dead_down, dead_zero = 0):
    if num - dead_zero < dead_up and num - dead_zero > -abs(dead_down):
        return dead_zero
    return num

if __name__ == "__main__":
    f = MovingAverageFilter()
    print(f.get_value(2))
    print(f.get_value(1))
    print(f.get_value(3))
    print(f.get_value(4))
    print(f.get_value(5))