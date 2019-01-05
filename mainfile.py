import numpy as np

GRID_SIZE = 0 
NUMBER_OF_CARS = 0
NUMBER_OF_OBSTACLES = 0
OBSTACLES_LIST = []
CAR_START_LOCATION_LIST = []
CAR_DESTINATION_LIST = []
POLICIES = []
COST_MATRIX = []
VALUE_MATRIX = []
RESULT = []
        
def main():
    readInputFile()
    createCostDict()
    createPolicyDict()
    moveCars()
    writeToOutputFile(RESULT)
    return


#function to read input to a file
def readInputFile():
    inputFileName = 'input3.txt'
    global GRID_SIZE,NUMBER_OF_CARS,NUMBER_OF_OBSTACLES,OBSTACLES_LIST,CAR_START_LOCATION_LIST,CAR_DESTINATION_LIST
    #open the file and read each line to get desired values
    srcFile = open(inputFileName,'r')
    GRID_SIZE = int(srcFile.readline().strip())
    NUMBER_OF_CARS  = int(srcFile.readline().strip())
    NUMBER_OF_OBSTACLES = int(srcFile.readline().strip())
    for i in range(NUMBER_OF_OBSTACLES):
       OBSTACLES_LIST.append(srcFile.readline().strip())
    
    for i in range(NUMBER_OF_CARS):
        CAR_START_LOCATION_LIST.append(srcFile.readline().strip())
    
    for i in range(NUMBER_OF_CARS):
        CAR_DESTINATION_LIST.append(srcFile.readline().strip())
     
    #close the source file 
    srcFile.close()
    return 

def moveCars():
    global CAR_START_LOCATION_LIST,CAR_DESTINATION_LIST,POLICIES,RESULT,GRID_SIZE
    for i in range(len(CAR_START_LOCATION_LIST)):
        print "Car number" + str(i)
        total = 0
        for j in range(10):
            pos = CAR_START_LOCATION_LIST[i]
            cost_iteration = 0
            np.random.seed(j)
            swerve = np.random.random_sample(1000000)
            k=0
            while pos != CAR_DESTINATION_LIST[i]:
                col,row = map(int,pos.split(","))
                move = POLICIES[i][row][col]
                if swerve[k] > 0.7:
                    if swerve[k] > 0.8:
                        if swerve[k] > 0.9:
                            move = turn_left(turn_left(move))
                        else:
                            move= turn_left(move)
                    else:
                        move = turn_right(move)
                #update position
                if move == 'N' and row-1 != -1:
                    pos = str(col)+","+str(row-1)
                    cost_iteration += COST_MATRIX[i][row-1][col]
                elif move == 'S' and row+1 < GRID_SIZE:
                    pos = str(col)+","+str(row+1)
                    cost_iteration += COST_MATRIX[i][row+1][col]
                elif move == 'E' and col+1 < GRID_SIZE:
                    pos = str(col+1)+","+str(row)
                    cost_iteration += COST_MATRIX[i][row][col+1]
                elif move == 'W' and col-1 != -1:
                    pos = str(col-1)+","+str(row)
                    cost_iteration += COST_MATRIX[i][row][col-1]
                k+=1
            print cost_iteration
            total+= cost_iteration
        RESULT.append(str(total/10)+"\n")
    return     

#function to return the direction if moved right from current direction
def turn_right(direction):
    if direction == 'N':
        return 'W'
    if direction == 'W':
        return 'S'
    if direction == 'S':
        return 'E'
    if direction == 'E':
        return 'N'
    '''
    if direction == 'N':
        return 'E'
    if direction == 'E':
        return 'S'
    if direction == 'S':
        return 'W'
    if direction == 'W':
        return 'N'
    '''
#function to return the direction if moved left from current direction
def turn_left(direction):
    if direction == 'N':
        return 'E'
    if direction == 'E':
        return 'S'
    if direction == 'S':
        return 'W'
    if direction == 'W':
        return 'N'
    '''
    if direction == 'N':
        return 'W'
    if direction == 'W':
        return 'S'
    if direction == 'S':
        return 'E'
    if direction == 'E':
        return 'N'
    '''
    
#function to create list of dictionary object with cost obtained in each cell for every car      
def createCostDict():
    global OBSTACLES_LIST,GRID_SIZE,COST_MATRIX,FINE,GAS,CAR_DESTINATION_LIST,POLICIES,VALUE_MATRIX,CAR_START_LOCATION_LIST
    for k in range(NUMBER_OF_CARS):
        cost_dict = np.full([GRID_SIZE, GRID_SIZE],-1)
        value_dict = np.full([GRID_SIZE, GRID_SIZE],-1.0)
        dest_col,dest_row = map(int,CAR_DESTINATION_LIST[k].split(','))
        cost_dict[dest_row][dest_col] = cost_dict[dest_row][dest_col]+100
        value_dict[dest_row][dest_col] = cost_dict[dest_row][dest_col]
        
        for key in OBSTACLES_LIST:
            obs_col,obs_row = map(int,key.split(','))
            cost_dict[obs_row][obs_col] = cost_dict[obs_row][obs_col]-100
            value_dict[obs_row][obs_col] = value_dict[obs_row][obs_col]-100
        delta,x = 0,0
        COST_MATRIX.append(cost_dict)
        print "COST MATRIX"
        print cost_dict
        prev_best = 0 
        while True:
            #delta = 0
            delta+=1
            value_dict = one_step_lookahead(k, value_dict)
            best_action_value = np.max(value_dict)
            x = max(x, np.abs(best_action_value - prev_best))
            prev_best = best_action_value 
            print "VALUE MATRIX"
            print value_dict
            print x        
            if delta >10: #< 0.1:
                VALUE_MATRIX.append(value_dict) 
                break
    return 

def one_step_lookahead(car_index,val_matrix):
    global GRID_SIZE,COST_MATRIX,CAR_DESTINATION_LIST,COST_MATRIX
    dest_col,dest_row = map(int,CAR_DESTINATION_LIST[car_index].split(','))
    start_col,start_row = map(int,CAR_START_LOCATION_LIST[car_index].split(','))
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if (i == dest_row and j == dest_col):
                    val_matrix[i][j] = str(COST_MATRIX[car_index][dest_row][dest_col])
            else:
                if (i == start_row and j == start_col): #no fuel cost in the start position as it is counted only when a movement is done
                    reward = 0
                else:
                    reward = COST_MATRIX[car_index][i][j]
                nVal = val_matrix[i-1][j] if (i-1 >= 0) else -1.0
                sVal = val_matrix[i+1][j] if (i+1<GRID_SIZE) else -1.0
                eVal = val_matrix[i][j+1] if (j+1<GRID_SIZE) else -1.0
                wVal = val_matrix[i][j-1] if (j-1>=0) else -1.0
                val_matrix[i][j] =  max(reward+0.9*(0.7*nVal+0.1*sVal+0.1*eVal+0.1*wVal),reward+0.9*(0.1*nVal+0.7*sVal+0.1*eVal+0.1*wVal),reward+0.9*(0.1*nVal+0.1*sVal+0.7*eVal+0.1*wVal),reward+0.9*(0.1*nVal+0.1*sVal+0.1*eVal+0.7*wVal))
                
    return val_matrix        

       
#function to create list of dictionary object with direction in each cell to reach the destination for every car
def createPolicyDict():
    global NUMBER_OF_CARS,GRID_SIZE,COST_MATRIX,CAR_DESTINATION_LIST,POLICIES,VALUE_MATRIX
    for k in range(NUMBER_OF_CARS):
        policy_dict = np.full([GRID_SIZE, GRID_SIZE],"00")
        dest_col,dest_row = map(int,CAR_DESTINATION_LIST[k].split(','))
        #policy_dict[dest_row][dest_col] = COST_MATRIX[k][dest_row][dest_col]
        dest_cost = COST_MATRIX[k][dest_row][dest_col]
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                if (i == dest_row and j == dest_col):
                    policy_dict[dest_row][dest_col] = str(COST_MATRIX[k][dest_row][dest_col])
                else:
                    nVal = VALUE_MATRIX[k][i-1][j] if (i-1 >= 0) else -111111
                    sVal = VALUE_MATRIX[k][i+1][j] if (i+1<GRID_SIZE) else -111111
                    eVal = VALUE_MATRIX[k][i][j+1] if (j+1<GRID_SIZE) else -111111
                    wVal = VALUE_MATRIX[k][i][j-1] if (j-1>=0) else -111111
                    maxValInd = np.argmax([nVal,sVal,eVal,wVal])
                    if maxValInd == 0:
                        policy_dict[i][j] = 'N'
                    elif maxValInd == 1:
                        policy_dict[i][j] = 'S'
                    elif maxValInd == 2:
                        policy_dict[i][j] = 'E'
                    else: 
                        policy_dict[i][j] = 'W' 
        POLICIES.append(policy_dict)  
    print POLICIES
    return
        
#function to write output to a file
def writeToOutputFile(content):
    ouputFileName = 'output.txt'
    with open(ouputFileName, 'w') as destFile:
        for val in content:
            destFile.write(val)
        destFile.close()
    return

if __name__== "__main__":
  main()
