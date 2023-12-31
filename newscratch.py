import numpy as np
import time
# import matplotlib.pyplot as plt
# from scipy import interpolate

# from geometry_msgs.msg import Quaternion, Point, Pose
# from tf.transformations import quaternion_from_euler, euler_from_quaternion

def add_node(path_arr, dx, dy, idx):
    assert (idx >= 1)
    if idx == path_arr.shape[1]:
        return -1
    
    path_arr[:, idx] = path_arr[:, idx - 1] + [dx, dy, 0]
    path_arr[2:, idx] = np.arctan(dy / dx)
    print("idx: ",idx ,path_arr[:, idx])
    return idx + 1

def wrapto_pi(ang_dev):
    return (ang_dev + 180) % (360) - 180
    
    ang_error = 0
    assert (-2 * 180 <= ang_dev <= 2 * 180)
    if ang_dev <= -180:
        ang_error = 2 * 180 + ang_dev
    elif ang_dev > 180:
        ang_error = ang_dev - 2 * 180
    else:
        ang_error = ang_dev
    
    return ang_error
    ang_error = ang_error / 180
    

def demo_sympy():
    import sympy as s
    curr_x = 1
    curr_y = 2
    x = s.Symbol("x")
    f = x ** 2
    a = s.Symbol("a")
    b = s.Symbol("b")
    d = (x - a) ** 2 + (f - b) ** 2
    # d = (x - 1) ** 2 + (f - 2) ** 2
    dp = s.diff(d, x)
    dpp = s.diff(dp, x)
    fp = s.diff(f, x)
    
    start = time.time()
    d_curr = d.subs(a, curr_x).subs(b, curr_y)
    dp_curr = dp.subs(a, curr_x).subs(b, curr_y)
    dpp_curr = dpp.subs(a, curr_x).subs(b, curr_y)
    
    # roots = s.Poly(dp_curr, x).nroots()
    roots = s.real_roots(dp_curr, x)
    clos_x = min(roots, key = lambda r : d_curr.subs(x, r).evalf())
    clos_y = f.subs(x, clos_x)
    print(clos_x.evalf(), clos_y.evalf())
    
    cur_x = float(1)
    cur_y = float(fp.subs(x, clos_x).evalf())
    
    print(np.arctan2(cur_x, cur_y))
    
    # print(s.RootOf(dp, 0))
    end = time.time()
    
    print(end - start)
    start = end
    
def demo_list_piecewise():
    import sympy as s
    prevT = time.time()
    t_var = s.Symbol("t")
    a_var = s.Symbol("a")
    num_segments = 1000
    x_func = s.Piecewise(*[(i * t_var + i + a_var, t_var <= i + 1) for i in range(num_segments)])
    nextT = time.time()
    print("create piece_wise: ", nextT - prevT)
    
    
    prevT = nextT
    x_lam = s.lambdify(t_var, x_func, "numpy")
    nextT = time.time()
    print("lambdify: ", nextT - prevT)
    
    
    prevT = nextT
    for i in range(1000):
        t = x_lam(i)
    nextT = time.time()
    print("eval: ", nextT - prevT)
    
    
    prevT = nextT
    xp = s.diff(x_func, t_var)
    nextT = time.time()
    print("diff: ", nextT - prevT)
    
    
    prevT = nextT
    xp_curr = xp.subs(a_var, 2)
    nextT = time.time()
    print(x_func)
    print("subs: ", nextT - prevT)
    
    
    np_x = lambda x : np.piecewise(x, [x <= i + 1 for i in range(num_segments)], [lambda x : i * x + i for i in range(num_segments)])
    xp_curr_lam = s.lambdify(t_var, xp_curr)
    from scipy.optimize import minimize_scalar
    prevT = time.time()
    result = minimize_scalar(xp_curr_lam, bounds=(-1000, 1000), method='bounded')
    t = result.x
    nextT = time.time()
    print("minimize_sclalar: ", nextT - prevT)
    

    x_func = s.Piecewise((0.20 * t_var + 0.8, t_var <= 1), (0.45 * t_var + 0.55, t_var <= 2), (-0.12 * t_var + 1.69, t_var <= 3), (1.33, True))

def scipy_interpolate():
    import numpy as np
    prevT = time.time()
    phi = np.linspace(0, 2.*np.pi, 20)
    r = 0.5 + np.cos(phi)         # polar coords
    x, y = r * np.cos(phi), r * np.sin(phi)    # convert to cartesian
    nextT = time.time()
    print("creation: ", nextT - prevT)

    from scipy.interpolate import splprep, splev
    prevT = nextT
    tck, u = splprep([x, y], k=1,s=0)
    new_points = splev(u, tck)
    nextT = time.time()
    print("splev: ", nextT - prevT)
    
    der_points = splev(u, tck, der=1)
    print("tck:\n", der_points)
    print("tck:\n", new_points)
    
    import matplotlib.pyplot as plt
    prevT = nextT
    fig, ax = plt.subplots()
    ax.plot(x, y, 'ro')
    ax.plot(new_points[0], new_points[1], 'r-')
    ax.plot(der_points[0], der_points[1], 'b-')
    nextT = time.time()
    print("plot: ", nextT - prevT)
    
    import scipy
    def distance_to_curve(point, tck):
        x, y = point
        def objective(t):
            curve_point = splev(t, tck)
            dist = np.sqrt((x - curve_point[0])**2 + (y - curve_point[1])**2)
            # print("curve_t", t, curve_point, dist)
            return dist

        result = scipy.optimize.minimize_scalar(objective, bounds=(0.5, 1), method="bounded")
        return result.x

    # Given point (x, y)
    given_point = (0.8, -0.5)

    prevT = time.time()
    # Find the closest point on the curve to the given point
    closest_point_t = distance_to_curve(given_point, tck)

    # Evaluate the B-spline curve at the closest point parameter
    closest_point_on_curve = splev(closest_point_t, tck)
    nextT = time.time()
    print("minimize: ", nextT - prevT)

    print("Given Point:", given_point)
    print("Closest Point on Curve:", closest_point_on_curve)
    # ax.plot(splev(0.5, tck)[0], splev(0.5, tck)[1], 'go')
    ax.plot(closest_point_on_curve[0], closest_point_on_curve[1], 'go')
    
    derivative_of_closest_point = splev(closest_point_t, tck, der=1)
    ax.plot(derivative_of_closest_point[0], derivative_of_closest_point[1], 'ro')
    ax.plot(given_point[0], given_point[1], 'bo')
    plt.show()
            

import threading
import time

nem = 3
nrum = int(100000/nem)
gr= np.array([0] * nrum)
def do_nothing(n):
    global gr
    sumr = 0
    for i in range(n):
        sumr += gr[i]
    return sumr
            
if __name__=="__main__":    
    
    # ts=[]
    # t=time.time()
    # # do_nothing(1000)
    # for i in range(nem):
    #     thread = threading.Thread(target=do_nothing, args=(nrum,))
    #     ts.append(thread)
    #     thread.start()
    # for x in ts:
    #     print(ts)
    #     x.join()
    
    # print(gr)
    # print(time.time()-t)
    # exit(0)

    # demo_sympy()
    # demo_list_piecewise()
    scipy_interpolate()
    exit(0)
    
    
    # points = [(3.28,0.00),(4.00,0.50),(4.40,1.0),(4.60,1.52),(5.00,2.5),(5.00,3.34),(4.70,3.8)]
    # points = points + [(4.50,3.96),(4.20,4.0),(3.70,3.90),(3.00,3.5),(2.00,2.9)]
    points = [[0.8, 0.3], [1.05, 1.45], [1.5, 1.45], [1.38, 1.15]]
    data = np.array(points)

    tck,u = interpolate.splprep(data.transpose(), s=0)
    # unew = np.arange(0, 1.01, 0.01)
    unew = np.arange(0, 1.01, 0.01)
    out = interpolate.splev(unew, tck)

    plt.figure()
    plt.plot(out[0], out[1], color='orange')
    plt.plot(data[:,0], data[:,1], 'ob')
    plt.show()
    exit(0)
    
    
    
    angles = np.arange(-405, 361, 45)
    for angle in angles:
        print(f"{angle} -> {wrapto_pi(angle)}")
    
    
    exit(0)
    path = np.zeros(shape=(3,4))
    idx = 1
    idx = add_node(path, 1, 1, idx)
    idx = add_node(path, 1, 2, idx)
    idx = add_node(path, 1, 3, idx)
    # idx = add_node(path, 1, 4, idx)
    print(path)
    
    
    poses = []
    for i in range(path.shape[1]):
        curr_pose = Pose()
        curr_pose.position.x = path[0, i]
        curr_pose.position.y = path[1, i]
        curr_pose.position.z = 0
        q = quaternion_from_euler(0,0, path[2, i])
        curr_pose.orientation.x = q[0]
        curr_pose.orientation.y = q[1]
        curr_pose.orientation.z = q[2]
        curr_pose.orientation.w = q[3]
        poses.append(curr_pose)
    
    print(poses)
    