import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import famsec as famsec
from comms import min_diff_pos_sorted, find_nearest

def preprocess_predicted(predicted_paths):
    predicted_states = [np.load(d, allow_pickle=True) for d in predicted_paths]
    for idx, x in enumerate(predicted_states):
        for iidx, y in enumerate(x):
            predicted_states[idx][iidx, 7] = len(y[7])
        predicted_states[idx] = predicted_states[idx].astype('float64')
    max_len = np.max([len(x) for x in predicted_states])
    preprocessed = np.zeros((max_len, 8)) * np.nan
    indexes = [8, 0, 1, 7]  #
    for t in range(max_len):
        data = []
        for i in indexes:
            d1 = []
            for pred in predicted_states:
                if t >= pred.shape[0]:
                    continue
                d1.append(pred[t, i])
            data.append([np.mean(d1), np.std(d1)])
        preprocessed[t] = np.array(data).flatten()
    #  [t, t_mu, x_mu, x_std, y_mu, y_std, obs_mu, obs_std]
    return preprocessed


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
        plt.plot(_x, y, color='black')
        plt.plot([min(_x), max(_x)], [0, 0], color='black')
        plt.scatter([_loc - _dist, _loc + _dist], [_model.pdf(_loc - _dist), _model.pdf(_loc + _dist)], c='red')
        plt.plot([_loc + _dist, _loc + _dist], [0, _model.pdf(_loc + _dist)], c='red')
        plt.plot([_loc - _dist, _loc - _dist], [0, _model.pdf(_loc - _dist)], c='red')
        plt.title("full:{:.2f}, SI:{:.2f}, std:{:.2f}".format(_model.cdf(_myclip_b), _si, _scale))
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
        #ax.hist(dpredicted, density=True)
        ax.plot([actual, actual], [0, p_actual], color='red', linewidth=3)
        #ax.plot(xx[smaller], p_distribution_smaller)
        ax.set_title('SI={:.2f}'.format(surprise))
    return surprise


class et_goa:
    def __init__(self):
        self.pred_paths = []
        self.data = []
        self.counter = 0
        self.sample_rate = 10

    def set_pred_paths(self, paths):
        self.pred_paths = paths

    def preprocess(self):
        predicted_states = [np.load(d, allow_pickle=True) for d in self.pred_paths]
        for idx, x in enumerate(predicted_states):
            for iidx, y in enumerate(x):
                predicted_states[idx][iidx,7] = len(y[7])
            predicted_states[idx] = predicted_states[idx].astype('float64')
        max_len = np.max([len(x) for x in predicted_states])
        preprocessed = np.zeros((max_len, 8)) * np.nan
        indexes = [8, 0, 1, 7]  #
        for t in range(max_len):
            data = []
            for i in indexes:
                d1 = []
                for pred in predicted_states:
                    if t >= pred.shape[0]:
                        continue
                    d1.append(pred[t, i])
                data.append([np.mean(d1), np.std(d1)])
            preprocessed[t] = np.array(data).flatten()
        #  [t, x_mu, x_std, y_mu, y_std, obs_mu, obs_std]
        self.counter = 0
        self.data = preprocessed

    def get_si(self, actual_x, actual_y, actual_obs, t=0.0, min_std=2):
        if self.counter >= len(self.data):
            return 0.0, None, None
        d_idx = min_diff_pos_sorted(self.data[:, 0], t)
        #print("NEW: ({:.2f}, {:.2f})".format(self.data[d_idx][2], self.data[d_idx][4]))

        d = self.data[self.counter]
        #print("OLD:({:.2f}, {:.2f})".format(d[2], d[4]))
        print("mu(x,y, obs): ({:.2f}, {:.2f}, {:.2f}) || act({:.2f}, {:.2f}, {:.2f})"
              .format(d[2], d[4], d[6], actual_x, actual_y, actual_obs))
        print("")
        # x position
        mux = d[2]
        stdx = d[3]
        si_x = gaussian_si_1d(pred_mu=mux, pred_std=stdx, actual=actual_x, plot=False, min_std=min_std)

        # y position
        muy = d[4]
        stdy = d[5]
        si_y = gaussian_si_1d(pred_mu=muy, pred_std=stdy, actual=actual_y, plot=False, min_std=min_std)

        # num objects
        mu = d[6]
        std = d[7]
        si_o = gaussian_si_1d(pred_mu=mu, pred_std=std, actual=actual_obs, plot=False, min_std=min_std)
        si = np.min([si_x, si_y, si_o])
        self.counter += self.sample_rate
        print('t: {:.2f}, si: {:.2f}'.format(d[0], si))
        return si, mux, muy

    def get_goa_times(self, time_cutoff, time_completed):
        predicted_states = [np.load(d, allow_pickle=True) for d in self.pred_paths]
        times = [d[-1, -2] + time_completed for d in predicted_states]
        print("GOA TIMES: ", times)
        distribution = [int(x < time_cutoff) for x in times]
        partition = np.array([-2, 0, 2])
        z_star = 2
        goa_time = famsec.assess_rollouts(distribution=distribution, bins=partition, z_star=z_star)
        return goa_time


if __name__ == '__main__':
    pred_paths = [r'C:\DATA\webots\rollout{}_state.npy'.format(x) for x in np.arange(0, 10)]
    #et_obj = et_goa()
    #et_obj.set_pred_paths(pred_paths)
    #et_obj.preprocess()
    #et_obj.get_goa_times(135, 0)
    #data = et_obj.data
    #et_obj.get_si(0, 0, 0, t=120.01)

    raw = [np.load(d, allow_pickle=True) for d in pred_paths]
    act = 0
    t = 0
    x_t = [x[t][0] for x in raw]
    y_t = [x[t][1] for x in raw]
    pred = x_t
    fig, ax = plt.subplots()
    kde_assessment(3, np.array(pred), ax=ax)
    plt.show()
