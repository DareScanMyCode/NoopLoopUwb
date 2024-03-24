# NOOPLOOP UWB protocol
## Introduction
1. Recv from `dev` and decode the msg(Only `NLink_LinkTrack_Node_Frame5` now)
2. Send to others what I received through `NLink_LinkTrack_User_Frame1`, with Payload `UserPayload1`:
| Data | Type | length | Description |
| :-- | :-- | :-- | -- |
| role_from | u8 | 1 | distance to role_to measured by role_from |
| role_to | u8 | 1 | distance to role_to measured by role_from |
| dis_raw | f32 | 4 | raw distance |
| dis_filtered | f32 | 4 | filtered distance(MovingAverage) |
3. Using `MovingAverageFilter' filter the distance
4. Raise two adi_matrixes `adj_matrix_raw` and `adj_matrix_filtered`

## How to use
1. Packages need
```bash
pip install pyserial
```
2. Use uwb in your code: 
```python
from handle_uwb import NoopLoopUWB
uwb1 = NoopLoopUWB('COM3', log_ON = False)
uwb2 = NoopLoopUWB('COM5', log_ON = False)
```
3. Get adj matrixes from `uwb`
```python
print(uwb1.adj_matrix_raw[role_from][role_to])
print(uwb1.adj_matrix_filtered[role_from][role_to])
```
4. How to stop threads: 
```python
while True:
    try:
        time.sleep(0.2)
    except KeyboardInterrupt:
        uwb1.should_stop = True
        uwb2.should_stop = True
```

## Notice
1. The adj matrixes are **NOT symmetric**, since maybe A measured a distance of 0.555m to B, B measured 0.666 to A due to the noise. `0` will get if no distance measured.