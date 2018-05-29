from gaft import GAEngine
from gaft.components import BinaryIndividual
from gaft.components import Population
from gaft.operators import TournamentSelection
from gaft.operators import UniformCrossover
from gaft.operators import FlipBitBigMutation
# Built-in best fitness analysis.
from gaft.analysis.fitness_store import FitnessStore
from gaft.analysis.console_output import ConsoleOutput
import blackboxfunc_ga

'''
The parameters that should be changed are
Input : search_domain : the search bounds in which you want to find the reward
        is_two_dimensional = False for a single dimension attribute and True for a two dimensional attribute.
         num_of_population : size of the population

Output : The explored_points python file and explored paths by the humanoid robot and the best x and f(x) points and a few plottings.
'''
search_domain = (-5, 5)
is_two_dimensional = False
fn = blackboxfunc_ga.BlackBox(search_domain, is_two_dimensional)
if is_two_dimensional is False:
    indv_template = BinaryIndividual(ranges=[search_domain], eps=0.001)
else:
    indv_template = BinaryIndividual(ranges=[search_domain, search_domain], eps=0.001)
# Define population.
population = Population(indv_template=indv_template, size=50).init()

# Create genetic operators.
#selection = RouletteWheelSelection()
selection = TournamentSelection()
crossover = UniformCrossover(pc=0.8, pe=0.5)
mutation = FlipBitBigMutation(pm=0.1, pbm=0.55, alpha=0.6)

# Create genetic algorithm engine.
# Here we pass all built-in analysis to engine constructor.
engine = GAEngine(population=population, selection=selection,
                  crossover=crossover, mutation=mutation,
                  analysis=[ConsoleOutput, FitnessStore])

'''
In the fitness function we need to change the function that we want and four options 
are given. Replace the function in the fitness to run the fitness on that function accordingly.
1)Ackley Function (fn.ackley_fn(x, y))
2)Binary_fn (fn.binary_fn(x,y))
3)Eesom Function (fn.eesom_fn(x,y))
4)Schaffer N.2 Function (fn.schaffer_N_2(x,y))
5)Schaffer N.4 Function (fn.schaffer_N_4(x,y))
The above functions are two dimensional functions.

Below is a one dimensional function that can be used to evaluate as a test function
1)Sinc Function (fn.sinc_fn(x,y))

'''

# Define fitness function.
@engine.fitness_register
def fitness(indv):
    if is_two_dimensional is False:
        #Only used in the function of
        x, = indv.solution
        return fn.sinc_fn(x)
    else:
        x, y = indv.solution
        return fn.ackley_fn(x, y)

if '__main__' == __name__:
    engine.run(ng=100)
