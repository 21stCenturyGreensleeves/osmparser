from shapely import linestrings
import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression
import random

def geometryProcess(curvatures):
    """
    :param curvatures:输入一个曲率数组
    :return: 返回
    """
    min_length = 2
    max_length = 100
    threshold1 = 0.01
    threshold2 = 0.02
    threshold3 = 1.0
    zero_threshold = 0.01

    segments = []
    s = 0

    while s < len(curvatures):
        best_e = s
        best_model = None
        best_params = []

        for e in range(s + min_length - 1, len(curvatures)):
            current_points = curvatures[s:e+1]
            X = np.array([p.x for p in current_points]).reshape(-1, 1)
            y = np.array([p.y for p in current_points])

            inliers1 = [p for p in current_points if abs(p.y) <= threshold1]

            best_n_inliers = []
            best_n_value = None
            for _ in range(20):  # 随机采样次数
                sample = random.choice(current_points)
                n = sample.y
                candidates = [p for p in current_points if abs(p.y - n) <= threshold2]
                if len(candidates) > len(best_n_inliers):
                    best_n_inliers = candidates
                    best_n_value = n

            inliers3 = []
            try:
                ransac = RANSACRegressor(
                    LinearRegression(),
                    residual_threshold=threshold3
                )
                ransac.fit(X, y)
                inlier_mask = ransac.inlier_mask_
                inliers3 = [current_points[i] for i, mask in enumerate(inlier_mask) if mask]
            except:
                pass

            model_candidates = [
                (len(inliers1), 1, 0),  # (匹配数, 模型类型, 参数)
                (len(best_n_inliers), 2, best_n_value),
                (len(inliers3), 3, (
                    ransac.estimator_.coef_[0],
                    ransac.estimator_.intercept_
                ))
            ]

            # 按匹配点数降序排序
            model_candidates.sort(reverse=True, key=lambda x: x[0])

            # 选择最优模型
            selected_model = model_candidates[0]
            if selected_model[0] >= min_length and selected_model[0] / len(current_points) > 0.6:
                # 处理近似零值的情况
                if selected_model[1] == 2 and abs(selected_model[2]) < zero_threshold:
                    selected_model = (selected_model[0], 1, 0)

                best_model = selected_model[1]
                best_params = selected_model[2]
                best_e = e

            # 生成线段描述
        if best_model:
            start_x = curvatures[s][0]
            end_x = curvatures[best_e][0]
            if best_model == 1:
                segments.append(('type1', start_x, end_x, 0))
            elif best_model == 2:
                segments.append(('type2', start_x, end_x, best_params))
            else:
                a, b = best_params
                segments.append(('type3', start_x, end_x, a, b))
            s = best_e + 1  # 移动到下一个未处理点
        else:
            s += 1  # 未找到合适模型，前进1个点

        return segments
