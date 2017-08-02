"""DSSM Safety measure computations
 Ref:     A Study on the Traffic Predictive Cruise Control Strategy with
          Downstream Traffic Information
 Authors: Sehyun Tak, Sunghoon Kim, Hwasoo Yeo"""


def compute_trans_dist(speed, acc, b_max, max_acc_variation, reaction_time):
    """Computes the distance traveled during transition. (Eqn. 2 of the paper)"""
    t_trans = (acc - b_max) / max_acc_variation
    trans_dist = (speed + acc * reaction_time + 0.25 * (acc + b_max) * t_trans) * t_trans
    # print 't_trans: {}, speed: {}, acc: {} reaction: {} b_max: {}'.format(t_trans, speed, acc, reaction_time,  b_max)
    return trans_dist


def compute_dssm_and_braking(spacing, speed, acc, b_max, max_acc_variation, reaction_time,
                             leader_speed, leader_acc, leader_b_max, leader_max_acc_variation):
    """Computes the dssm value and corresponding brake value. Eqns. 2,3,4 of th paper"""
    d_trans_self = compute_trans_dist(speed, acc, b_max, max_acc_variation, reaction_time)
    # By setting reaction time to 0.0 for leader, we remove the a*\tau part from the equation:
    d_trans_lead = compute_trans_dist(leader_speed, leader_acc, leader_b_max, leader_max_acc_variation, 0.0)
    K = -spacing + speed * reaction_time + 0.5 * acc * reaction_time ** 2 - d_trans_lead + d_trans_self  # Eqn. (2)
    b = leader_b_max * ((speed + acc * reaction_time) ** 2) / (2.0 * K * leader_b_max + leader_speed ** 2)  # Eqn. (3)
    dssm = b / b_max  # Eqn. (4)
    # print 'dssm: {} d_tr_self: {} d_tr_lead: {} K: {} b: {}'.format(dssm, d_trans_self, d_trans_lead, K, b)
    return dssm, b
