import numpy as np
import scipy.stats as stats


class Metrics:
    def __init__(self):
        pass

    @staticmethod
    def logistic(x, x0=0, L=2, k=0.3, vert_shift=0):
        """
        Logistic with x0=0 is a sigmoid.
        https://en.wikipedia.org/wiki/Logistic_function
        :param k:   logistic growth rate
        :param x:   x value
        :param x0:  x value of midpoint
        :param L:   height of logistic
        :return: logistic transform of x
        """
        return L / (1 + np.exp(-k * (x - x0))) - vert_shift

    @staticmethod
    def tanh(x):
        return np.tanh(x)


class FaMSeC:
    def __init__(self):
        pass

    @staticmethod
    def generalized_outcome_assessment(outcome_dist, outcome_partitions, z_star):
        """
        Calculate generalized Outcome Assessment metric for a competency bound on an ordinal set of data using the
        ratio of moments.

        :param outcome_dist:        Singular outcomes of trajectories, organized s.t. larger values are better outcomes.
        :param outcome_partitions:  Lower limits on equivalence classes of outcomes.
        :param z_star:              Threshold for which we desire outcome bins above (inclusive).
        :return:                    GOA for user specified z_star in the range [0,1]
        """
        # get # of bins
        num_bins = len(outcome_partitions)

        if z_star < 1 or z_star > num_bins:
            raise Exception("z* must be an integer between 1 and number of bins")

        # ranked z-domain
        z_domain = []
        for i in range(1, num_bins):
            z_i = []
            z_i.append(
                [
                    val
                    for val in outcome_dist
                    if outcome_partitions[i - 1] < val <= outcome_partitions[i]
                ]
            )
            z_domain.append(z_i)

        # estimate the probabilities in each bin
        p_z = []
        for current_bin in z_domain:
            p_z.append(len(current_bin[0]) / len(outcome_dist))

        # compute UPM/LPM
        d_lpm = 0
        d_upm = 0

        for i in range(1, len(z_domain) + 1):
            if i < z_star:
                d_lpm += (z_star - i) * p_z[i - 1]
            elif i >= z_star:
                d_upm += (i - z_star + 1) * p_z[i - 1]

        # compute GOA
        if d_lpm == 0:
            outcome_assessment = 1
        elif d_upm == 0:
            outcome_assessment = 0
        else:
            outcome_assessment = Metrics.logistic(
                d_upm / d_lpm, vert_shift=1
            )  # spacial.expit(d_upm/d_lpm-1)
        return outcome_assessment

    @staticmethod
    def outcome_assessment(reward_dist, r_star, swap=False):
        """
        Calculate the rewards based Outcome Assessment metric using the difference of moments.

        :param reward_dist:     distribution of rewards
        :param r_star:          minimum threshold for acceptable rewards
        :return:                OA for user specified r_star in the range [0,1]
        """
        c = 1
        a = 1
        num_data = len(reward_dist)
        rwd_prob = np.zeros_like(reward_dist, dtype=float)
        for i in np.unique(reward_dist):
            idx = np.where(reward_dist == i)
            rwd_prob[idx] = 1 / num_data

        # UPM/LPM
        # source:
        upm_data = np.where(reward_dist >= r_star)
        lpm_data = np.where(reward_dist < r_star)

        if swap:
            upm_data, lpm_data = lpm_data, upm_data

        upm = np.sum((np.abs(reward_dist[upm_data]) * rwd_prob[upm_data]) ** c)
        lpm = np.sum((np.abs(reward_dist[lpm_data]) * rwd_prob[lpm_data]) ** a)
        # compute GOA
        if lpm == 0:
            o = 1
        elif upm == 0:
            o = 0
        else:
            o = Metrics.logistic(upm - lpm, L=1, k=0.09)

        return o

    @staticmethod
    def model_quality(predicted_dist, action_obs):
        """
        Calculate the model quality metric based on predicted and actual data

        :param predicted_dist:  distribution of predicted values
        :param action_obs:      single actual observation
        :return:                the surprise of the observation given the predictions in the range [0, 1]
        """
        kernel = stats.gaussian_kde(predicted_dist, bw_method="scott")
        xx = np.linspace(min(predicted_dist) - 10, max(predicted_dist) + 10, 500)
        smaller = np.where(kernel(xx) < kernel(action_obs))
        p_actual = kernel(action_obs)[0]
        p_distribution = kernel(xx)
        p_distribution_smaller = p_distribution[smaller]

        surprise = np.trapz(p_distribution_smaller, xx[smaller])

        if True:
            import matplotlib.pyplot as plt

            plt.plot(xx, p_distribution)
            plt.hist(predicted_dist, density=True)
            plt.plot([action_obs, action_obs], [0, p_actual], color="red", linewidth=3)
            plt.plot(xx[smaller], p_distribution_smaller)
            plt.title("SI={:.2f}".format(surprise))
            plt.show()
        return surprise

    @staticmethod
    def pt_outcome_assessment(self, distribution, r_star):
        alpha = 0.88
        a = 0.5  # 0.88
        b = 0.88
        l = 2.25
        g = 0.69
        gamma = 0.69

        def v_function(R):
            """
            transformed reward function
            "humans tend to perceive the slope between large rewards to be smaller than the slope between small ones"
            :param R: reward
            :return:
            """
            if R >= 0:
                return R**a
            else:
                return -l * np.abs(R) ** b
                # return -1 * np.abs(R) ** 1

        def w_function(p):
            """
            transformed probability function
            "humans tend to underweight large probabilities and overweight smaller ones"
            :param p:
            :return:
            """
            return (p**g) / (p**g + (1 - p) ** g) ** (1 / g)
            # return (p ** 1) / (p ** 1 + (1 - p) ** 1) ** (1 / 1)

        def cpt_single_point(p, u):
            return w_function(p) * v_function(u)

        hist, bins = np.histogram(a=distribution, density=True, bins=10)
        bin_means = []
        print("less: ", len(distribution[distribution <= 0]))
        for i in range(len(bins) - 1):
            bin_means.append(np.mean(bins[i : i + 2]))
        hist = hist / np.sum(hist)
        utility = 0.0
        for i in range(len(bin_means)):
            utility += cpt_single_point(bin_means[i], hist[i])

        return Metrics.logistic(utility)


def assess_rollouts(distribution, bins, z_star):
    goa = FaMSeC.generalized_outcome_assessment(distribution, bins, z_star)
    return goa


def assessment_to_label(raw_assessment):
    if raw_assessment < 0.25:
        return "very bad", "red"
    elif raw_assessment < 0.4:
        return "bad", "#C34A2C"
    elif raw_assessment < 0.6:
        return "fair", "yellow"
    elif raw_assessment < 0.75:
        return "good", "#6CC417"
    else:
        return "very good", "green"
