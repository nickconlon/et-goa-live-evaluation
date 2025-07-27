import famsec as famsec
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
from world_model import WorldModel

def gaussian_si_1d(pred_mu, pred_std, actual, min_std=1, plot=False):
    _myclip_a = min(actual - 10, pred_mu - 10)
    _myclip_b = max(actual + 10, pred_mu + 10)
    _loc = pred_mu
    _scale = np.maximum(pred_std, min_std)
    _a, _b = (_myclip_a - _loc) / _scale, (_myclip_b - _loc) / _scale
    _model = stats.norm(loc=_loc, scale=_scale)
    _x = np.linspace(_myclip_a, _myclip_b, num=500)
    _dist = abs(_loc - actual)
    _si = _model.cdf(_loc - _dist) + (1 - _model.cdf(_loc + _dist))
    if plot:
        y = _model.pdf(_x)
        plt.plot(_x, y, color="black")
        plt.plot([min(_x), max(_x)], [0, 0], color="black")
        plt.scatter(
            [_loc - _dist, _loc + _dist],
            [_model.pdf(_loc - _dist), _model.pdf(_loc + _dist)],
            c="red",
        )
        plt.plot([_loc + _dist, _loc + _dist], [0, _model.pdf(_loc + _dist)], c="red")
        plt.plot([_loc - _dist, _loc - _dist], [0, _model.pdf(_loc - _dist)], c="red")
        plt.title(
            "full:{:.2f}, SI:{:.2f}, std:{:.2f}".format(
                _model.cdf(_myclip_b), _si, _scale
            )
        )
        plt.xlim([min(_x), max(_x)])
        plt.tight_layout()
        plt.pause(0.1)
        plt.clf()
    return _si


def kde_assessment(actual, predicted, ax=None):
    dpredicted = predicted[~np.isnan(predicted)]
    if len(dpredicted) == 0:
        return 0.0
    if len(np.unique(dpredicted)) == 1:
        dpredicted = np.array([dpredicted[0] - 5, dpredicted[0], dpredicted[0] + 5])
    kernel = stats.gaussian_kde(dpredicted, bw_method=100)
    xx = np.linspace(min(dpredicted) - 5, max(dpredicted) + 5, 500)
    smaller = np.where(kernel(xx) < kernel(actual))
    p_actual = kernel(actual)[0]
    p_distribution = kernel(xx)
    p_distribution_smaller = p_distribution[smaller]

    surprise = np.trapz(p_distribution_smaller, xx[smaller])
    if ax is not None:
        ax.clear()
        ax.plot(xx, p_distribution)
        # ax.hist(dpredicted, density=True)
        ax.plot([actual, actual], [0, p_actual], color="red", linewidth=3)
        # ax.plot(xx[smaller], p_distribution_smaller)
        ax.set_title("SI={:.2f}".format(surprise))
    return surprise


class et_goa:
    def __init__(self, world_model: WorldModel):
        self.pred_paths = []
        self.data = []
        self.counter = 0
        self.sample_rate = 1
        self.world_model = world_model

    def get_si(self, actual_x, actual_y, t=0.0, min_std=2.0):
        state_dist = self.world_model.get_distribution_at_time(t)

        d = [
            state_dist[0, 0],  # mu(t)
            state_dist[0, 1],  # std(t)
            state_dist[1, 0],  # mu(x)
            state_dist[1, 1],  # std(x)
            state_dist[2, 0],  # mu(y)
            state_dist[2, 1],  # std(y)
        ]
        print(
            "mu(t, x, y): ({:.2f}, {:.2f}, {:.2f}) || act({:.2f}, {:.2f}, {:.2f})".format(
                d[0], d[2], d[4], t, actual_x, actual_y
            )
        )
        print()
        # x position
        mux = d[2]
        stdx = d[3]
        si_x = gaussian_si_1d(
            pred_mu=mux, pred_std=stdx, actual=actual_x, plot=False, min_std=min_std
        )

        # y position
        muy = d[4]
        stdy = d[5]
        si_y = gaussian_si_1d(
            pred_mu=muy, pred_std=stdy, actual=actual_y, plot=False, min_std=min_std
        )

        si = np.min([si_x, si_y])  # TODO turned off sensor Model Quality
        self.counter += self.sample_rate
        print("t: {:.2f}, si: {:.2f}".format(d[0], si))
        return si, mux, muy

    def get_goa_times(self, time_cutoff):
        times = [d[-1, -2] for d in self.world_model.get_distribution()]
        print("GOA TIMES: ", times)
        distribution = [int(x < time_cutoff) for x in times]
        partition = np.array([-2, 0, 2])
        z_star = 2
        goa_time = famsec.assess_rollouts(
            distribution=distribution, bins=partition, z_star=z_star
        )
        return goa_time


if __name__ == "__main__":
    pred_paths = [
        r"C:\DATA\webots\rollout{}_state.npy".format(x) for x in np.arange(0, 10)
    ]
    # et_obj = et_goa()
    # et_obj.set_pred_paths(pred_paths)
    # et_obj.preprocess()
    # et_obj.get_goa_times(135, 0)
    # data = et_obj.data
    # et_obj.get_si(0, 0, 0, t=120.01)

    raw = [np.load(d, allow_pickle=True) for d in pred_paths]
    act = 0
    t = 0
    x_t = [x[t][0] for x in raw]
    y_t = [x[t][1] for x in raw]
    pred = x_t
    fig, ax = plt.subplots()
    kde_assessment(3, np.array(pred), ax=ax)
    plt.show()
