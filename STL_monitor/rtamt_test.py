import sys
import rtamt

def monitor():
    spec = rtamt.STLDiscreteTimeSpecification()
    spec.name = 'Bounded-response Request-Grant'

    spec.declare_var('req', 'float')
    spec.declare_var('gnt', 'float')
    spec.declare_var('out', 'float')

    spec.spec = 'out = always((req>=3) implies (eventually[0:5](gnt>=3)))'

    try:
        spec.parse()
        spec.update(0, [('req', 0.1), ('gnt', 0.3)])
        spec.update(1, [('req', 0.45), ('gnt', 0.12)])
        spec.update(2, [('req', 0.78), ('gnt', 0.18)])
        nb_violations = spec.sampling_violation_counter # nb_violations = 0
    except rtamt.STLParseException as err:
        print('STL Parse Exception: {}'.format(err))
        sys.exit()

if __name__ == '__main__':
    # Process arguments
    monitor()
