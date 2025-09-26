import numpy as np
import sympy as sp
from calc_jacobian import calcJacobian


def distToLine():
    """ compute the distance of a point qb to a line qiqj, and find its gradient wrt qi, qj """
    # define vars
    px, py = sp.symbols('px, py')
    vx, vy = sp.symbols('vx, vy')
    qix, qiy = sp.symbols('qix, qiy')
    p = sp.Matrix([px, py])
    v = sp.Matrix([vx, vy])
    qi = sp.Matrix([qix, qiy])

    # write cost function
    u = (qb - qi).cross(qj - qi)
    fu = u.dot(u)
    v = (qj - qi)
    fv = v.dot(v)
    f = fu / fv
    f = sp.simplify(f)
    print(f)

    # find jacobian
    df_dqi = calcJacobian([f], [qix, qiy]).transpose()
    df_dqi = sp.simplify(df_dqi)
    print(df_dqi)
    df_dqj = calcJacobian([f], [qjx, qjy]).transpose()
    df_dqj = sp.simplify(df_dqj)


def distToLine2():
    """ compute the distance of a point qb to a line qiqj, and find its gradient wrt qi, qj """
    # define vars
    px, py = sp.symbols('px, py')
    vx, vy = sp.symbols('vx, vy')
    qix, qiy = sp.symbols('qix, qiy')
    p = sp.Matrix([px, py])
    v = sp.Matrix([vx, vy])
    qi = sp.Matrix([qix, qiy])

    # write cost function
    u = (qb - qi).cross(qj - qi)
    print(sp.simplify(u))
    return

    fu = u.dot(u)
    v = (qj - qi)
    fv = v.dot(v)
    f = fu / fv
    f = sp.simplify(f)
    print(f)

    # find jacobian
    df_dqi = calcJacobian([f], [qix, qiy]).transpose()
    df_dqi = sp.simplify(df_dqi)
    print(df_dqi)
    df_dqj = calcJacobian([f], [qjx, qjy]).transpose()
    df_dqj = sp.simplify(df_dqj)


def distToView():
    px, py = sp.symbols('px, py')
    vx, vy = sp.symbols('vx, vy')
    qix, qiy = sp.symbols('qix, qiy')
    p = sp.Matrix([px, py])
    v = sp.Matrix([vx, vy])
    qi = sp.Matrix([qix, qiy])
    

    # dist to view
    d = (qi - p) - ((qi - p).dot(v)) * v
    D = d.dot(d)
    dD_dqi = calcJacobian([D], [qix, qiy]).transpose()
    dD_dqi = sp.simplify(dD_dqi)
    sp.pprint(dD_dqi)
    print(dD_dqi)

    dd_dqi = calcJacobian([d[0], d[1]], [qix, qiy])
    dd_dqi = sp.simplify(dd_dqi)
    print(dd_dqi)

if __name__ == "__main__":
    #distToLine2()
    distToView()
