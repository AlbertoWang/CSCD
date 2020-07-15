import argparse
def start_py_point_cloud():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-s', '--showpc', action='store_true')
    group.add_argument('-s', '--showpc', action='store_true')