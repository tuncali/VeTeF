"""Defines various classes and tool functions used for simulation configurations."""
import sys
import os
import numpy as np
import subprocess
import pickle
import datetime
import struct
from communication_client import CommunicationClient
from sim_data import SimData
from webots_road import WebotsRoad
from webots_vehicle import WebotsVehicle
from simulation_message_interface import SimulationMessageInterface

# sys.path.append("D:/cma-1.1.7")
# sys.path.append("C:/Users/getun/Documents/cma-1.1.7")
# import cma


class RunConfig(object):
    """RunConfig class defines configuration for execution of a simulation."""
    def __init__(self):
        self.VUT_CONTROLLER = 'simple_controller'
        self.DUMMY_CONTROLLER = 'dummy_vhc_nn_control'
        self.DUMMY_POS_LAT = 3.5
        self.DUMMY_POS_LON = 0.0
        self.simulation_run_mode = SimData.SIM_TYPE_FAST_NO_GRAPHICS


class SimulationConfig(object):
    """SimulationConfig class defines configuration for a simulation.
    A simulation configuration also contains run configuration array."""
    def __init__(self, world_no):
        # self.world_file = '..\\..\\..\\worlds\\test_world_{}.wbt'.format(world_no)
        self.world_file = '../worlds/test_world_{}.wbt'.format(world_no)
        self.server_port = 10020 + world_no
        self.server_ip = '127.0.0.1'
        self.run_config_arr = []
        self.sim_duration_ms = 50000
        self.sim_step_size = 10
        self.simulator_instance_pid = None

    def insert_run_config(self, run_config):
        """Inserts a run configuration to the simulation configuration."""
        import copy
        self.run_config_arr.append(copy.deepcopy(run_config))


class ExperimentConfig(object):
    """ExperimentConfig class contains parallelization / log etc. information for running a series of simulations."""
    def __init__(self):
        self.NUM_WEIGHTS = 418
        self.WEIGHTS_MIN = -5.0
        self.WEIGHTS_MAX = 5.0
        self.MAX_ITERATIONS = 1
        self.INITIAL_SIGMA = 3.0
        self.NUM_OF_PARALLEL_WEBOTS = 1
        self.LOG_FILE_PREFIX = '.\\logs\\' + datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_")
        self.WEBOT_RESTART_THRESHOLD = 110
        self.SIMULATION_RUN_FUNC = None
        self.sim_config_arr = []
        self.cma_results = None
        self.POP_SIZE = None


class WithExtraArgs(object):
    """WithExtraArgs class is used to pass extra arguments to the parallelized function calls."""
    def __init__(self, func, *args):
        self.func = func
        self.args = args

    def __call__(self, idx):
        return self.func(self.args[0][idx], self.args[1][idx])


def create_road_object():
    road_obj = WebotsRoad()
    return road_obj


def create_vehicle_object():
    vehicle_obj = WebotsVehicle()
    return vehicle_obj


def start_webots(world_file, minimized):
    if sys.platform == 'win32':  # Windows
        cmd = 'C:\\Program Files\\Webots\\msys64\\mingw64\\bin\\webots'
    elif sys.platform == 'darwin':  # Mac OS
        cmd = '/Applications/Webots.app/webots'
    else:  # Linux
        cmd = 'webots'
    if minimized:
        params = ['--mode=fast', '--minimize', '--batch', world_file]
        simulator_instance = subprocess.Popen([cmd, params[0], params[1], params[2], params[3]])
    else:
        params = ['--mode=fast', '--batch', world_file]
        simulator_instance = subprocess.Popen([cmd, params[0], params[1], params[2]])

    return simulator_instance


def start_webots_from_path(world_file, minimized, path):
    cmd = path
    if minimized:
        params = ['--mode=fast', '--minimize', '--batch', world_file]
        simulator_instance = subprocess.Popen([cmd, params[0], params[1], params[2], params[3]])
    else:
        params = ['--mode=fast', '--batch', world_file]
        simulator_instance = subprocess.Popen([cmd, params[0], params[1], params[2]])

    return simulator_instance


def kill_process_id(pid):
    if sys.platform == 'win32':  # Windows
        subprocess.call(['taskkill', '/F', '/T', '/PID', str(pid)])
    # else:
    #     subprocess.call(['kill', str(pid)])


def kill_webots(simulator_instance):
    try:
        kill_process_id(simulator_instance.pid)
    except:
        pass


def kill_webots_pid(pid):
    try:
        kill_process_id(pid)
    except:
        pass


def get_weights_from_file(results_file_name):
    exp_conf = pickle.load(open(results_file_name, 'rb'))
    return exp_conf.cma_results[0]  # Best solution is at index 0. See Python CMA Documentation.


def get_experiment_config_from_file(file_name):
    exp_conf = pickle.load(open(file_name, 'rb'))
    return exp_conf


def save_results_to_file(res, results_file_name):
    pickle.dump(res, open(results_file_name, 'wb'))


def run_cma_experiment(experiment_config):
    simulator_instance = start_webots(experiment_config.sim_config_arr[0].world_file, True)
    first_sample = np.random.uniform(low=experiment_config.WEIGHTS_MIN,
                                     high=experiment_config.WEIGHTS_MAX,
                                     size=(1, experiment_config.NUM_WEIGHTS))
    first_sample = first_sample.tolist()[0]
    es = cma.CMAEvolutionStrategy(first_sample, experiment_config.INITIAL_SIGMA)
    SimLogger = cma.CMADataLogger(experiment_config.LOG_FILE_PREFIX).register(es)
    es.optimize(run_simulation,
                iterations=experiment_config.MAX_ITERATIONS,
                logger=SimLogger,
                verb_disp=True,
                args=([experiment_config.sim_config_arr[0]]))
    res = es.result()
    results_file_name = experiment_config.LOG_FILE_PREFIX + 'results.dat'
    save_results_to_file(res, results_file_name)
    kill_webots_pid(simulator_instance.pid)
    # es.plot()


def run_cma_experiment_parallel(experiment_config):
    import cma
    import time
    import multiprocessing as mp

    num_of_parallel = experiment_config.NUM_OF_PARALLEL_WEBOTS
    first_sample = np.random.uniform(low=experiment_config.WEIGHTS_MIN,
                                     high=experiment_config.WEIGHTS_MAX,
                                     size=(1, experiment_config.NUM_WEIGHTS))
    first_sample = first_sample.tolist()[0]
    if experiment_config.POP_SIZE is None:
        es = cma.CMAEvolutionStrategy(first_sample,
                                      experiment_config.INITIAL_SIGMA,
                                      {'maxiter': experiment_config.MAX_ITERATIONS})
    else:
        es = cma.CMAEvolutionStrategy(first_sample,
                                      experiment_config.INITIAL_SIGMA,
                                      {'maxiter': experiment_config.MAX_ITERATIONS,
                                       'popsize': experiment_config.POP_SIZE})
    if num_of_parallel > 0:
        pool_size = num_of_parallel
    else:
        pool_size = 1

    SimLogger = cma.CMADataLogger(experiment_config.LOG_FILE_PREFIX).register(es)
    pool = mp.Pool(len(experiment_config.sim_config_arr))
    sim_count = 0
    simulator_instance_arr = []
    while not es.stop():
        if sim_count == 0:
            simulator_instance_arr = []
            for sim_config in experiment_config.sim_config_arr:
                simulator_instance_arr.append(start_webots(sim_config.world_file, True))
                sim_config.simulator_instance_pid = simulator_instance_arr[-1].pid
        X = es.ask()
        time.sleep(0.2 * pool_size)  # Wait 0.2 seconds per webots instance to run
        if pool_size < es.popsize:
            f_values = [0.0] * es.popsize
            cur_index = 0
            last_index = cur_index + pool_size
            while cur_index < es.popsize:
                last_index = min(last_index, es.popsize)
                time.sleep(0.05 * pool_size)  # Wait additional 0.05 seconds per webots instance to revert
                f_values[cur_index:last_index] = pool.map_async(WithExtraArgs(experiment_config.SIMULATION_RUN_FUNC,
                                                                              X[cur_index:last_index],
                                                                              experiment_config.sim_config_arr),
                                                                range(last_index - cur_index)).get()
                cur_index = last_index
                last_index = cur_index + pool_size
        else:
            time.sleep(0.2 * pool_size)  # Wait 0.2 seconds per webots instance to revert
            f_values = pool.map_async(WithExtraArgs(experiment_config.SIMULATION_RUN_FUNC,
                                                    X,
                                                    experiment_config.sim_config_arr),
                                      range(len(X))).get()
        sim_count += es.popsize
        # We regularly kill and run webots because of webots memory leak problem (Webots: 8.5.3)
        if sim_count >= experiment_config.WEBOT_RESTART_THRESHOLD:
            print('Simulation count: {}... Restarting Webots instances.'.format(sim_config))
            for simulator_instance in simulator_instance_arr:
                kill_webots_pid(simulator_instance.pid)
            simulator_instance_arr = []
            sim_count = 0
            time.sleep(0.5)  # Wait for all webots instances to die
        es.tell(X, f_values)
        es.disp()
        SimLogger.add()
        try:
            SimLogger.add(es, modulo=bool(SimLogger.modulo))
        except:
            None
        print('Iteration {}/{} is complete. Best value: {}.'.format(es.countiter,
                                                                    experiment_config.MAX_ITERATIONS,
                                                                    es.best.f))

    pool.close()
    pool.join()
    for simulator_instance in simulator_instance_arr:
        kill_webots_pid(simulator_instance.pid)
    res = es.result()
    experiment_config.cma_results = res
    results_file_name = experiment_config.LOG_FILE_PREFIX + 'results.dat'
    save_results_to_file(experiment_config, results_file_name)
    print('Experiment is complete. Best value: {}.'.format(res[1]))
    print('Results saved to : {}'.format(results_file_name))
    # es.plot()


def run_results(results_file_name):
    exp_config = get_experiment_config_from_file(results_file_name)
    for i in range(len(exp_config.sim_config_arr[0].run_config_arr)):
        exp_config.sim_config_arr[0].run_config_arr[i].simulation_run_mode = SimData.SIM_TYPE_REAL_TIME
    simulator_instance = start_webots(exp_config.sim_config_arr[0].world_file, False)
    weights = exp_config.cma_results[0]
    # print weights
    exp_config.SIMULATION_RUN_FUNC(weights, exp_config.sim_config_arr[0])
    kill_webots_pid(simulator_instance.pid)
