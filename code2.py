import cplex
from cplex.exceptions import CplexError
import sys

# create an instance of a Cplex problem
prob = cplex.Cplex()

# set the maximum number of threads to use
prob.parameters.threads.set(4)

# set the maximum amount of time to solve the problem
prob.parameters.timelimit.set(60)

# set the objective sense to minimize
prob.objective.set_sense(prob.objective.sense.minimize)

# create a list to hold the variable names
var_names = []

print("Site-Dependent Multi-Trip Periodic Vehicle Routing Problem")
print("We have 10 input files")
files=["sdmtpvrp1.txt","sdmtpvrp2.txt","sdmtpvrp3.txt","sdmtpvrp4.txt","sdmtpvrp5.txt","sdmtpvrp6.txt","sdmtpvrp7.txt","sdmtpvrp8.txt","sdmtpvrp9.txt","sdmtpvrp10.txt"]
print("Choosing the first file to work upon")
file=0 #change the value of this file variable(0-9) to change the file of your choice

with open("input/"+files[0], 'r') as f:

    # read the first line of the file
    n, t, C, P, m, tt = map(float, f.readline().split())

    # read the depot information
    depot_x, depot_y, lt = map(float, f.readline().split())

    # create a list to store the vehicle information
    vehicles = []

    # read the information for each vehicle
    for i in range(int(m)):
        p, Q, D, c = map(int, f.readline().split())
        vehicles.append((p, Q, D, c))

    # create a list to store the customer information
    customers = []

    # read the information for each customer
    for i in range(int(n)):
        line =f.readline()
        line = line.strip().split()
        line_len=len(line)
        rem_len=line_len-6
        x = float(line[0])
        y = float(line[1])
        q = int(line[2])
        ut = int(line[3])
        freq = int(line[4])
        a = int(line[5])
        list1 = list(map(int, line[6:6+a]))
        e = int(line[7+a])
        list2 = list(map(int, line[7+a:]))
        customers.append((x, y, q, ut, freq, a, list1, e, list2))

    # create a list to store the delivery-day pattern information
    delivery_patterns = []

    #read the information for each delivery-day pattern
    for i in range(int(C)):
        line =f.readline()
        line = line.strip().split()
        freq = int(line[0])
        days = list(map(int, line[1:]))
        delivery_patterns.append((freq, days))


# create the decision variables
for i in range(int(n)):
    for j in range(int(n)):
        if i != j:
            for k in range(int(m)):
                for r in range(int(t)):
                    var_name = "x_" + str(i) + "_" + str(j) + "_" + str(k) + "_" + str(r)
                    var_names.append(var_name)
                    prob.variables.add(obj=[customers[i][j] * vehicles[k][3]], types=[prob.variables.type.binary], names=[var_name])

# add the constraints to ensure that each customer is visited exactly once
for i in range(int(n)):
    constr = cplex.SparsePair()
    for j in range(int(n)):
        if i != j:
            for k in range(int(m)):
                for r in range(int(t)):
                    index = var_names.index("x_" + str(i) + "_" + str(j) + "_" + str(k) + "_" + str(r))
                    constr.add(index, 1)
    prob.linear_constraints.add(lin_expr=[constr], senses=["E"], rhs=[1.0])

# add the constraints to ensure that each route starts and ends at the depot
for k in range(int(m)):
    for r in range(int(t)):
        constr1 = cplex.SparsePair()
        constr2 = cplex.SparsePair()
        for i in range(int(n)):
            index1 = var_names.index("x_0_" + str(i) + "_" + str(k) + "_" + str(r))
            constr1.add(index1, 1)
            index2 = var_names.index("x_" + str(i) + "_0_" + str(k) + "_" + str(r))
            constr2.add(index2, 1)
        prob.linear_constraints.add(lin_expr=[constr1], senses=["E"], rhs=[1.0])
        prob.linear_constraints.add(lin_expr=[constr2], senses=["E"], rhs=[1.0])

# add the constraints to limit the maximum number of deliveries per vehicle
for k in range(int(m)):
    for r in range(int(t)):
        constr = cplex.SparsePair()
        for i in range(int(n)):
            for j in range(int(n)):
                if i != j:
                    index = var_names.index("x_" + str(i) + "_" + str(j) + "_" + str(k) + "_" + str(r))
                    constr.add(index, 1)
        prob.linear_constraints.add(lin_expr=[constr], senses=["L"], rhs=[vehicles[k][1]])

# add the constraints to limit the maximum duration of each route
for k in range(int(m)):
    for r in range(int(t)):
        constr = cplex.SparsePair()
        for i in range(int(n)):
            for j in range(int(n)):
                if i != j:
                    index = var_names.index("x_" + str(i) + "_" + str(j) + "_" + str(k) + "_" + str(r))
                    constr.add(index, customers[i][j])
                    prob.linear_constraints.add(lin_expr=[var_names[i] for i in range(len(var_names))],senses=['E' for i in range(len(var_names))],rhs=[1 for i in range(len(var_names))])

#Adding constraints (6) and (7) (vehicle capacity constraints) for each vehicle type and each day of the period
for p in range(P):
    for k in range(t):
        # retrieve the maximum capacity of vehicle type p
        Qp = vehicles[p][1]
        # retrieve the set of customers that can be visited on day k
        Sk = set()
        for c in range(C):
            if k+1 in delivery_patterns[c][1]:
                Sk = Sk.union(set(customers[c][6]))
        # create a list of constraints to enforce the vehicle capacity constraint for each route
        for route in vehicles[p][k]:
            route_vars = []
            for c in route:
                route_vars += [vehicles[(p, c, i, k)] for i in range(len(customers[c][6]))]
                prob.linear_constraints.add(lin_expr=[route_vars],
                senses=['L'],
                rhs=[Qp])

#Adding constraints (8) and (9) (vehicle duration constraints) for each vehicle type and each day of the period
for p in range(P):
    for k in range(t):
        # retrieve the maximum duration of vehicle type p
        Dp = vehicles[p][2]
        # retrieve the set of customers that can be visited on day k
        Sk = set()
        for c in range(C):
            if k+1 in delivery_patterns[c][1]:
                Sk = Sk.union(set(customers[c][6]))
                # create a list of constraints to enforce the vehicle duration constraint for each route
                for route in vehicles[p][k]:
                    route_vars = []
                    for c in route:
                        route_vars += [vehicles[(p, c, i, k)] for i in range(len(customers[c][6]))]
                        prob.linear_constraints.add(lin_expr=[route_vars],
                        senses=['L'],
                        rhs=[Dp])

#Adding constraints (10) and (11) (vehicle load constraints) for each vehicle type and each day of the period
for p in range(P):
    for k in range(t):
        # retrieve the maximum capacity of vehicle type p
        Qp = vehicles[p][1]
        # retrieve the set of customers that can be visited on day k
        Sk = set()
        for c in range(C):
            if k+1 in delivery_patterns[c][1]:
                Sk = Sk.union(set(customers[c][6]))
                # create a list of constraints to enforce the vehicle load constraint for each customer
                for c in Sk:
                    load_vars = []
                    for i in range(len(customers[c][6])):
                        for j in range(i+1):
                            if j == i:
                                load_vars += [vehicles[(p, c, i, k)]]
                            else:
                                load_vars += [vehicles[(p, c, j, k)] + vehicles[(p, c, i, k)]]
                                prob.linear_constraints.add(lin_expr=[load_vars],
                                senses=['L'],
                                rhs=[customers[c][2]])


# add objective function to the model
prob.objective.set_sense(prob.objective.sense.minimize)

# solve the problem
prob.solve()

# print the solution status
print("Solution status = ", prob.solution.get_status())

# print the optimal objective value
print("Optimal value = ", prob.solution.get_objective_value())

# print the optimal solution
for v in prob.variables():
    if v.varValue > 0:
        print(v.name, "=", v.varValue)