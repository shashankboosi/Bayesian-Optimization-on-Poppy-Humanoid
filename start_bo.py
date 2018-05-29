import bo
class setup_bayes:
    def __init__(self, is_two_dimensional=False):
        self.is_two_dimensional = is_two_dimensional
        self.num_iter = num_iter
        bo.Bayesian_optimization_main(num_iter=num_iter, is_two_dimensional=self.is_two_dimensional)
'''
The parameters that should be changed are
Input : search_domain : the search bounds in which you want to find the reward
        is_two_dimensional = False for a single dimension attribute and True for a two dimensional attribute.

Output : The explored paths by the humanoid robot and the best x and f(x) points and a few plottings.
'''
is_two_dimensional=False
num_iter = 25
start = setup_bayes(num_iter=num_iter, is_two_dimensional=is_two_dimensional)
