package intsyscw1;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Random;
import java.util.Stack;

/**
 * This is my solution for the assignment. It does not intent to solve the
 * problem but just to implement the 4 search methods.
 *
 * @author Alexandru Amarandei Stanescu aas1u16
 */
public class Main {

    // These are the variables in which the number of visited nodes are stored
    private int dfsRecursiveNodesVisited, bfsNodesVisited, dfsIterativNodesVisited, astarNodesVisited, deepeningNodesVisited, astarNodesTakenIntoAccount, astarNodesWithVerTakenIntoAccount, astarNodesWithVerVisited;
    // In these variables the current position of the agent is stored and it's start position
    private Coordinates agent, agentS;
    // The arrays with the current coordonates of the blocks, their start positions, final positions and obstacle coordonates
    private ArrayList<Coordinates> currentCoord, startCoord, finalCoord, obstacleCoord;
    // Stacks for dfs and iterative deepening
    private Stack dfsStack, itDPStack;
    // My move X and moveY vectors
    // They represent the change of the position of the agent 
    private int[] moveX, moveY;
    // ArrayList to store a shuffeled version of numbers from 1 to 4 to help at DFS
    private ArrayList<Integer> shuffel;
    // Boolean values to see if a solution has been found.
    // Because DFS and ITDP are recursive, then the values have to be global
    private boolean solFoundDFSR, solFoundItDP, smallSolutionFoundDFSR;
    // The board size and the number of blocks on the board
    private int boardSizeX, boardSizey, blockNumber;
    // Boolean to see if the program is testing
    private boolean testing = false;

    /**
     * This function creates the default start state, from the cw.
     */
    public void assignStartState() {
        boardSizeX = 4;
        boardSizey = 4;
        moveX = new int[]{1, 0, -1, 0};
        moveY = new int[]{0, 1, 0, -1};
        currentCoord = new ArrayList<>();
        agentS = new Coordinates(1, 4);
        finalCoord = new ArrayList<>();
        finalCoord.add(new Coordinates(3, 2));
        finalCoord.add(new Coordinates(2, 2));
        finalCoord.add(new Coordinates(1, 2));

        startCoord = new ArrayList<>();
        startCoord.add(new Coordinates(1, 1));
        startCoord.add(new Coordinates(1, 2));
        startCoord.add(new Coordinates(1, 3));
        agent = new Coordinates();
        dfsStack = new Stack();
        itDPStack = new Stack();
        solFoundDFSR = false;
        shuffel = new ArrayList<>();
        shuffel.add(0);
        shuffel.add(1);
        shuffel.add(2);
        shuffel.add(3);
        blockNumber = 3;
    }

    /**
     * This function creates a random state with a given column size, row size
     * and number of blocks.
     *
     * @param columnSize Indicates the column size
     * @param rowSize Indicates the row size
     * @param numberOfBlocks Indicates the number of blocks
     */
    public void assignRandomState(int columnSize, int rowSize, int numberOfBlocks) {
        boardSizeX = columnSize;
        boardSizey = rowSize;
        moveX = new int[]{1, 0, -1, 0};
        moveY = new int[]{0, 1, 0, -1};
        currentCoord = new ArrayList<>();

        obstacleCoord = new ArrayList<>();
        Random random = new Random();
        Coordinates newPosition = new Coordinates();
        //Randomly generate final ditinct positions  
        finalCoord = new ArrayList<>();
        for (int i = 0; i < numberOfBlocks; i++) {
            newPosition.x = random.nextInt(rowSize) + 1;
            newPosition.y = random.nextInt(columnSize) + 1;
            boolean isContained = false;
            for (int j = 0; j < finalCoord.size(); j++) {
                if (finalCoord.get(j).equals(newPosition)) {
                    isContained = true;
                }
            }
            if (!isContained) {
                finalCoord.add(new Coordinates(newPosition.x, newPosition.y));
            } else {
                i--;
            }
        }
        //Generate random distinct start states 
        startCoord = new ArrayList<>();
        for (int i = 0; i < numberOfBlocks; i++) {
            newPosition.x = random.nextInt(rowSize) + 1;
            newPosition.y = random.nextInt(columnSize) + 1;
            boolean isContained = false;
            for (int j = 0; j < startCoord.size(); j++) {
                if (startCoord.get(j).equals(newPosition)) {
                    isContained = true;
                }
            }
            if (!isContained) {
                startCoord.add(new Coordinates(newPosition.x, newPosition.y));
            } else {
                i--;
            }
        }
        //Create a new start coordinate for agent that is not one of the blocks
        boolean isContained = true;
        while (isContained) {
            isContained = false;
            newPosition.x = random.nextInt(rowSize) + 1;
            newPosition.y = random.nextInt(columnSize) + 1;
            for (int j = 0; j < startCoord.size(); j++) {
                if (startCoord.get(j).equals(newPosition)) {
                    isContained = true;
                }
            }
            agentS = new Coordinates(newPosition.x, newPosition.y);
        }
        agent = new Coordinates();
        dfsStack = new Stack();
        itDPStack = new Stack();
        solFoundDFSR = false;
        shuffel = new ArrayList<>();
        shuffel.add(0);
        shuffel.add(1);
        shuffel.add(2);
        shuffel.add(3);
        blockNumber = numberOfBlocks;

    }

    /**
     * Adds a number of obstacles to the game. We add obstacles only if there is
     * room for them.
     *
     * @param numberOfObstacles Indicates number of obstacles
     */
    public void addObstacles(int numberOfObstacles) {
        if (numberOfObstacles + blockNumber * 2 + 1 < boardSizeX * boardSizey) {
            obstacleCoord = new ArrayList<>();
            Random random = new Random();
            Coordinates newObstacle = new Coordinates();
            //Generate new distinct obstacles that are not on any final position or start position
            for (int i = 0; i < numberOfObstacles; i++) {
                newObstacle.x = random.nextInt(boardSizeX) + 1;
                newObstacle.y = random.nextInt(boardSizey) + 1;
                boolean isContained = false;
                for (int j = 0; j < startCoord.size(); j++) {
                    if (startCoord.get(j).equals(newObstacle)) {
                        isContained = true;
                    }
                    if (finalCoord.get(j).equals(newObstacle)) {
                        isContained = true;
                    }

                }
                for (int j = 0; j < obstacleCoord.size(); j++) {
                    if (obstacleCoord.get(j).equals(newObstacle)) {
                        isContained = true;
                    }
                }
                if (agentS.equals(newObstacle)) {
                    isContained = true;
                }

                if (!isContained) {
                    obstacleCoord.add(new Coordinates(newObstacle.x, newObstacle.y));
                } else {
                    i--;
                }
            }
        }
    }

    /**
     * This function restores everything to the initial state. Use this before
     * every iteration of any search algorithm.
     */
    public void reassignStartValues() {
        //Reassigns the coordinates
        currentCoord.clear();
        for (int i = 0; i < startCoord.size(); i++) {
            currentCoord.add(startCoord.get(i));
        }
        //And then the agent
        agent.getValuesFrom(agentS);
    }

    /**
     * Starts DFS. This version will try endlessly to find a lucky solution
     * without overflowing. Not recommended.
     */
    public void doDFSR() {
        boolean error = true;
        smallSolutionFoundDFSR = false;
        //while (error || !smallSolutionFoundDFSR) {
        while (error) {
            error = false;
            try {
                dfsRecursiveNodesVisited = 0;
                solFoundDFSR = false;
                dfsStack.clear();
                dfsStack.push(agentS);
                reassignStartValues();
                dfsRecursive();
            } catch (StackOverflowError e) {
                error = true;
            }
        }
    }

    /**
     * Starts BFS.
     *
     */
    public void doBFS() {
        bfsNodesVisited = 0;
        reassignStartValues();
        bfs(agentS);
    }

    /**
     * Starts DFS implemented iteratively.
     */
    public void doDFSI() {

        smallSolutionFoundDFSR = false;
        //If you wish to obtain a human readable solution unccoment the code of do while
        do {
            dfsIterativNodesVisited = 0;
            reassignStartValues();
            dfsIterativ();
        } while (dfsIterativNodesVisited > 100);

    }

    /**
     * Start iterative deepening. Because a start state can be a final state, we
     * need to put the printing function here
     */
    public void doITDeep() {
        deepeningNodesVisited = 0;
        reassignStartValues();
        //Checks if startState is the same as the final state
        solFoundItDP = checkIfCurrentStateIsSolution();
        //Starts at lvl 0
        int currentDepthLevel = 0;
        itDPStack.push(agentS);

        while (!solFoundItDP) {
            reassignStartValues();
            iterativeExploration(currentDepthLevel);
            //Increments level in case a solution was not found
            currentDepthLevel++;
        }
        printITDPSolution();

    }

    /**
     * Starts AStar search.
     */
    public void doAStar() {
        astarNodesVisited = 0;
        astarNodesTakenIntoAccount = 0;
        reassignStartValues();
        astar();

    }

    /**
     * Starts my AStart search.
     */
    public void doAStarWithVer() {
        astarNodesWithVerVisited = 0;
        astarNodesWithVerTakenIntoAccount = 0;
        reassignStartValues();
        astarWithVerification();

    }

    /**
     * Starts the verifying process.
     */
    public void start() {
        //Choose a way to verify the algorithms
        //Comment or unccoment code

        // This will assign the default state
        assignStartState();
        //This will assign a random state
        //assignRandomState(3, 3, 3);
        //This will add obstacles
        //addObstacles(0);

        reassignStartValues();
        System.out.println(startCoord);
        System.out.println(finalCoord);
        System.out.println(agentS);

        System.out.println("Total distance on a " + boardSizeX + " by " + boardSizey + " with " + blockNumber + " blocks:" + distanceFromFinalState());
        //System.out.println("\nCalculating DFS recursive number of nodes:");
        //doDFSR();
        System.out.println("\nCalculating DFS iterativ number of nodes:");
        doDFSI();

        System.out.println("\nCalculating BFS number of nodes:");
        doBFS();

        System.out.println("\nCalculating iterative deepening number of nodes:");
        doITDeep();
        System.out.println("\nCalculating A* number of nodes:");
        doAStar();
        System.out.println("\nCalculating A* with verification number of nodes:");
        doAStarWithVer();

        //Unccoment this if you want to test 
        //tester(10,100);
    }

    /**
     * Use start() to start the class.
     *
     * @param args
     */
    public static void main(String[] args) {
        Main main = new Main();
        main.start();
    }

    /**
     * This function test all matrixes from 1X1 to sizeXsize with blocks from 1
     * to size * (size-1) a number of times each.
     *
     * @param size The max size of the matrix
     * @param numberOfTest Number of tests
     */
    public void tester(int size, int numberOfTest) {
        int maxSize = size;
        int distance[] = new int[size * size * size * 2];
        int times[] = new int[size * size * size * 2];
        // We make this true so we do not print useless information
        testing = true;
        //We start calculating and save the distance for data testing
        for (int i = 2; i <= maxSize; i++) {
            for (int b = 1; b < i * (i - 1); b++) {
                //Reinitialitze the vectors
                for (int j = 0; j < distance.length; j++) {
                    distance[j] = 0;
                    times[j] = 0;
                }
                //Start testing
                for (int trys = 0; trys < numberOfTest; trys++) {
                    int dist;

                    assignRandomState(i, i, b);
                    reassignStartValues();
                    dist = distanceFromFinalState();
                    //I strongly reccomend using this tester to test each algorithm separetly
                    //Uncomment wanted algorithm
                    /*
                    doDFSI();
                    distance[dist] += dfsIterativNodesVisited;
                    times[dist]++;
                     */
 /*
                    doBFS();
                    distance[dist] += bfsNodesVisited;
                    times[dist]++;
                     */
 /*
                    doITDeep();
                    distance[dist] += deepeningNodesVisited;
                    times[dist]++;
                     */
 /*
                    doAStar();
                    distance[dist] += astarNodesVisited;
                    times[dist]++;
                     */
 /*
                    astarNodesWithVerVisited = 0;
                    doAStarWithVer();
                    distance[dist] += astarNodesWithVerVisited;
                    times[dist]++;
                     */

                }
                //Print the new states
                for (int j = 1; j < distance.length; j++) {
                    if (times[j] != 0) {
                        System.out.println(i + " " + i + " " + b + " " + j + "   " + (distance[j] / times[j]));
                    }
                }
            }
        }
        testing = false;
    }

    /**
     * Simple implementation of a recursive dfs.
     */
    public void dfsRecursive() {
        //We check if current state is a solution
        if (checkIfCurrentStateIsSolution()) {
            printDFSSolution();
            solFoundDFSR = true;
        } else {
            //If not then we explore the next nodes
            dfsRecursiveNodesVisited++;
            // We create a new shuffel so we don't get stuck
            Collections.shuffle(shuffel);
            int[] currentShuffle = new int[4];
            //Then we save it in memory
            for (int i = 0; i < 4; i++) {
                currentShuffle[i] = shuffel.get(i);
            }
            //For every possible move
            for (int i = 0; i < 4; i++) {
                //Move agent
                agent.x = agent.x + moveX[currentShuffle[i]];
                agent.y = agent.y + moveY[currentShuffle[i]];
                //Check if the new coordinates are in the matrix
                if (CoordinatesInBouds(agent) && !solFoundDFSR) {
                    //If they are then create a new agent and push it down into to stack to save the solution path
                    Coordinates agentDFS = new Coordinates(agent.x, agent.y);
                    dfsStack.push(agentDFS);
                    //Move the piece
                    changeBlockLocation(new Coordinates(agent.x - moveX[currentShuffle[i]], agent.y - moveY[currentShuffle[i]]), agent);
                    //Go into recursion
                    dfsRecursive();
                    //When we come back we have to pop the last element from the solution stack 
                    //But we need to check if the stack is empty because a solution might have been found
                    if (!dfsStack.empty()) {
                        dfsStack.pop();
                    }
                    //We change the block back
                    changeBlockLocation(agent, new Coordinates(agent.x - moveX[currentShuffle[i]], agent.y - moveY[currentShuffle[i]]));
                }
                //And then we change the agent back
                agent.x = agent.x - moveX[currentShuffle[i]];
                agent.y = agent.y - moveY[currentShuffle[i]];
            }
        }

    }

    /**
     * An implementation of a bfs with a queue.
     *
     * @param start The start Coordinates for the BFS
     */
    public void bfs(Coordinates start) {
        //First initialize the queue with the start state.
        Queue<State> queue = new LinkedList<>();
        State startState = new State(currentCoord, start);
        queue.add(startState);
        State currentState;
        boolean solFoundBFS = false;

        while (!queue.isEmpty()) {
            //Get the head of the queue
            currentState = queue.poll();
            //Check if it is a solution
            if (checkIfCurrentStateIsSolution(currentState)) {
                printBFSSolution(currentState);
                queue.clear();
                solFoundBFS = true;
            } else {
                //If not explore the node
                bfsNodesVisited++;
                agent = currentState.agent;
                for (int i = 0; i < 4; i++) {
                    //Try to make a move with the agent
                    Coordinates agentBFS = new Coordinates();

                    agentBFS.x = agent.x + moveX[i];
                    agentBFS.y = agent.y + moveY[i];
                    //Then check if the move is in bounds
                    if (CoordinatesInBouds(agentBFS) && !solFoundBFS) {
                        //If it is in bounds, then we add it to the queue by creating a copy of the current state
                        ArrayList<Coordinates> newBlockCoord = new ArrayList<>();
                        for (int j = 0; j < currentState.blocks.size(); j++) {
                            newBlockCoord.add(currentState.blocks.get(j));
                        }
                        //Then we check if the new state is not going backwards
                        State newState = new State(newBlockCoord, agentBFS);
                        if (!newState.equals(currentState.previosState)) {
                            changeBlockLocation(agent, agentBFS, newState);
                            newState.previosState = currentState;
                            //If not, we add it to the queue and link it to the current state
                            queue.add(newState);
                        }
                    }
                }
            }
        }
    }

    /**
     * An implementation of a dfs iterativ. We implement it using a LIFO
     * structure, a stack.
     */
    public void dfsIterativ() {

        Stack<DfsNode> dfsItNodes = new Stack<>();
        Stack<Coordinates> dfsItSol = new Stack<>();
        //We start with the start node
        DfsNode startNode = new DfsNode(agentS);
        dfsItNodes.push(startNode);
        boolean solfoundDFSIt = false;
        while (!dfsItNodes.isEmpty() && !solfoundDFSIt) {
            DfsNode dfsItAgent = dfsItNodes.pop();

            //We check if the current state is a solution
            if (checkIfCurrentStateIsSolution()) {
                dfsItSol.push(dfsItAgent.Coordinates);
                solfoundDFSIt = true;
                printDFSItSolution(dfsItSol);
            } else {
                //If not, then we explore the node
                //If it's the first time we explore this node then we add it to the solution path
                if (dfsItAgent.currentNode == 0) {
                    dfsItSol.push(dfsItAgent.Coordinates);
                    dfsIterativNodesVisited++;
                }
                
                //If the node is exhausted and has no more new nodes to return then we pop it from the solution and we go revert the moves
                if (dfsItAgent.currentNode == 4) {
                    //Eliminate from solution
                    dfsItSol.pop();
                    //Change back the blocks
                    changeBlockLocation(agent, dfsItAgent.Coordinates);
                    //Reasign the agent
                    agent.x = dfsItAgent.Coordinates.x;
                    agent.y = dfsItAgent.Coordinates.y;
                } else {
                    //If the node is not exhausted, then we get it's next possible agent
                    DfsNode newAgentDFSIt = dfsItAgent.nextDFSNode();
                    //If we can still use the current node we push it back to the stack
                    if (dfsItAgent.currentNode < 5) {
                        dfsItNodes.push(dfsItAgent);
                    }
                    //We check if the new node has coordonates in bounds
                    if (CoordinatesInBouds(newAgentDFSIt.Coordinates) && !solfoundDFSIt) {
                        //If it does then we move it and push it into the stack
                        changeBlockLocation(dfsItAgent.Coordinates, newAgentDFSIt.Coordinates);
                        agent.x = newAgentDFSIt.Coordinates.x;
                        agent.y = newAgentDFSIt.Coordinates.y;
                        dfsItNodes.push(newAgentDFSIt);
                    }
                }
            }

        }
    }

    /**
     * An implementation of iterative exploration. If is the same as a DFS
     * recursive but it will stop after it reaches a certain level.
     *
     * @param lvl Current level of recursion. If becomes smaller the deeper we
     * advance.
     */
    public void iterativeExploration(int lvl) {

        if (lvl >= 0) {
            //If bottom level is not reached
            deepeningNodesVisited++;
            Collections.shuffle(shuffel);
            int[] currentShuffle = new int[4];
            //We create a shuffel for it
            for (int i = 0; i < 4; i++) {
                currentShuffle[i] = shuffel.get(i);
            }
            //Then for all possible new states
            for (int i = 0; i < 4; i++) {
                //We try to move the agent in that direction
                Coordinates newCoordinates = new Coordinates();
                Coordinates oldCoordinates = new Coordinates();
                oldCoordinates.x = agent.x;
                oldCoordinates.y = agent.y;
                newCoordinates.x = agent.x + moveX[currentShuffle[i]];
                newCoordinates.y = agent.y + moveY[currentShuffle[i]];
                agent.x = newCoordinates.x;
                agent.y = newCoordinates.y;
                if (CoordinatesInBouds(newCoordinates) && !solFoundItDP) {
                    //If it can be moved then we consider it part of the solution
                    itDPStack.push(newCoordinates);
                    //We move the blocks accordingly
                    changeBlockLocation(oldCoordinates, newCoordinates);
                    //Then check if the current form is a solution
                    if (checkIfCurrentStateIsSolution()) {
                        solFoundItDP = true;
                    } else {
                        //Then we go into recursion
                        iterativeExploration(lvl - 1);
                        //When we come back we pop the currentNode from the stack if a solution was not found
                        if (!itDPStack.empty() && !solFoundItDP) {
                            itDPStack.pop();
                        }
                        //Then we change the blocks back
                        changeBlockLocation(newCoordinates, oldCoordinates);
                    }
                }
                //And reassign the agent to it's old coordonates
                agent.x = oldCoordinates.x;
                agent.y = oldCoordinates.y;
            }

        }
    }

    /**
     * This function returns the distance from the final state of a state. It
     * does this by adding the Manhattan distance from the current coordinates
     * of the blocks and their final position.
     *
     * @param state State to get the distance on
     * @return The distance
     */
    public int distanceFromFinalState(State state) {
        int distance = 0;
        for (int i = 0; i < state.blocks.size(); i++) {
            distance += Coordinates.manhattanDistance(state.blocks.get(i), finalCoord.get(i));
        }
        return distance;
    }

    /**
     * This function calculates the distance of the current state from it's
     * final state. It does this by adding the Manhattan distance from the
     * current coordinates of the blocks and their final position.
     *
     * @return The distance
     */
    public int distanceFromFinalState() {
        int distance = 0;
        for (int i = 0; i < currentCoord.size(); i++) {
            distance += Coordinates.manhattanDistance(currentCoord.get(i), finalCoord.get(i));
        }
        return distance;
    }

    /**
     * An implementation for AStar search. It uses a priority queue to store all
     * the nodes
     *
     * h(x) = distanceFromFinalState(); g(x) = numberOfSteps();
     */
    public void astar() {
        //We assing the first values
        PriorityQueue<ASState> pq = new PriorityQueue<>();
        ASState startState = new ASState(startCoord, agentS);

        startState.steps = 0;
        startState.value = distanceFromFinalState(startState);
        pq.add(startState);
        while (!pq.isEmpty()) {
            //Get the head of the queue which has the smalles possible value
            ASState currentState = pq.poll();
            //We check if the current state is solved
            if (checkIfCurrentStateIsSolution(currentState)) {
                printAStarSolution(currentState);
                pq.clear();
            } else {
                //If not then we explore all the possible states
                astarNodesVisited++;
                for (int i = 0; i < 4; i++) {
                    Coordinates newCoord = new Coordinates();
                    newCoord.x = currentState.agent.x + moveX[i];
                    newCoord.y = currentState.agent.y + moveY[i];
                    //If the new position of the agent is in bound
                    if (CoordinatesInBouds(newCoord)) {

                        astarNodesTakenIntoAccount++;
                        ArrayList<Coordinates> newBlockCoord = new ArrayList<>();
                        for (int j = 0; j < currentState.blocks.size(); j++) {
                            newBlockCoord.add(currentState.blocks.get(j));
                        }
                        ASState newState = new ASState(newBlockCoord, newCoord);

                        newState.steps = currentState.steps + 1;
                        //We link it to it's predecessor
                        newState.previosState = currentState;
                        //Then we change the blocks coordinates
                        changeBlockLocation(currentState.agent, newCoord, newState);
                        //Then we calculate it's value in order to put him into the queue
                        newState.value = distanceFromFinalState(newState) + newState.steps;
                        //And then we add it to the queue
                        pq.add(newState);
                    }
                }
            }

        }

    }

    /**
     * My implementation of A*. It verifies if a current state has been already
     * in the queue and removes duplicates from the queue at each step.
     */
    public void astarWithVerification() {
        boolean solfoundAStarVer = false;
        Queue<ASState> queue = new LinkedList<>();
        ASState startState = new ASState(startCoord, agentS);
        int minValue;
        int relaxation = 0;
        queue.add(startState);
        //While we didn't find a solution
        while (!solfoundAStarVer) {
            //If the queue is empty it means we ran into a dead end
            if (queue.isEmpty()) {
                //So we relax the condition
                relaxation++;
                queue.add(startState);
                //If we find that the relaxation is bigger than the actual maximum possible difference in value
                if (relaxation > blockNumber * boardSizeX * boardSizey) {
                    //We declare it an unsolvable puzzle
                    System.err.print("Unsolvable puzzle");
                    solfoundAStarVer = true;
                    astarNodesWithVerVisited = -1;
                }
            }
            //Then we assign a new minim Value - relaxation to not overflow on int
            minValue = Integer.MAX_VALUE - relaxation - 1;
            ArrayList<ASState> tempList = new ArrayList<>();
            //While the queue has elements to explore
            while (!queue.isEmpty()) {

                ASState currentState = queue.poll();
                //We check if the current state is a solution
                if (checkIfCurrentStateIsSolution(currentState)) {
                    printAStarWithVerSolution(currentState);
                    queue.clear();
                    solfoundAStarVer = true;
                } else {
                    //If not we explore it
                    astarNodesWithVerVisited++;
                    for (int i = 0; i < 4; i++) {
                        Coordinates agentAStar = new Coordinates();
                        agentAStar.x = currentState.agent.x + moveX[i];
                        agentAStar.y = currentState.agent.y + moveY[i];
                        if (CoordinatesInBouds(agentAStar) && !solfoundAStarVer) {
                            astarNodesWithVerTakenIntoAccount++;
                            ArrayList<Coordinates> newBlockCoord = new ArrayList<>();
                            for (int j = 0; j < currentState.blocks.size(); j++) {
                                newBlockCoord.add(currentState.blocks.get(j));
                            }
                            ASState newState = new ASState(newBlockCoord, agentAStar);
                            newState.steps = currentState.steps + 1;
                            changeBlockLocation(currentState.agent, agentAStar, newState);
                            boolean previousState = false;
                            State tempState = currentState;
                            //After we create the new state we check with all it's predecessors if it has beed repeated
                            while (tempState != null) {
                                if (newState.equals(tempState)) {
                                    previousState = true;
                                }
                                tempState = tempState.previosState;
                            }

                            if (!previousState) {
                                //If not then we compute it's value and link it
                                newState.previosState = currentState;
                                newState.value = distanceFromFinalState(newState) + newState.steps;
                                //Then we check if it can add it to the next wave of nodes
                                if (newState.value >= minValue && newState.value <= minValue + relaxation) {
                                    //If we can then we check if it's not a duplicate
                                    boolean isAlreadyIn = false;
                                    for (int j = 0; j < tempList.size(); j++) {
                                        if (newState.equals(tempList.get(j))) {
                                            isAlreadyIn = true;
                                        }
                                    }
                                    //If not, then we add
                                    if (!isAlreadyIn) {
                                        tempList.add(newState);
                                    }
                                }
                                //If we find a smaller value, we eliminate those values from the queue that are bigger than the new value + relaxation
                                if (minValue > newState.value) {
                                    for (int j = 0; j < tempList.size(); j++) {
                                        if (tempList.get(j).value > newState.value + relaxation) {
                                            tempList.remove(j);
                                        }
                                    }
                                    //Then we add it to the list and change the minimum
                                    tempList.add(newState);
                                    minValue = newState.value;
                                }

                            }
                        }
                    }
                }
            }
            //Then we swap the list with the queue
            for (int i = 0; i < tempList.size(); i++) {
                queue.add(tempList.get(i));
            }
            tempList.clear();
        }
    }

    /**
     * Prints a ITDP solution if no testing is in place.
     */
    public void printITDPSolution() {
        if (testing == false) {
            System.out.println(deepeningNodesVisited);
            while (!itDPStack.empty()) {
                System.out.print(itDPStack.pop() + " ");
            }
        } else {
            itDPStack.clear();
        }
    }

    /**
     * Prints a AStart solution if no testing is in place from the current
     * state.
     *
     * @param state Current state to print from
     */
    public void printAStarSolution(ASState state) {
        if (testing == false) {
            System.out.println(astarNodesVisited);
            System.out.println("Nodes taken into account: " + astarNodesTakenIntoAccount);
            System.out.println(state.steps);
            State printState = state;
            while (printState != null) {
                System.out.print(printState.agent);
                printState = printState.previosState;
            }
        }
    }

    /**
     * Prints a AStartWithVer solution if no testing is in place from the
     * current state.
     *
     * @param state Current state
     */
    public void printAStarWithVerSolution(ASState state) {
        if (testing == false) {
            System.out.println(astarNodesWithVerTakenIntoAccount + " " + astarNodesWithVerVisited);
            System.out.println(state.steps);
            State printState = state;
            while (printState != null) {
                System.out.print(printState.agent);
                printState = printState.previosState;
            }
        }
    }

    /**
     * Prints a DFSR solution if no testing is in place.
     */
    public void printDFSSolution() {
        if (testing == false) {
            if (dfsRecursiveNodesVisited < 100) {
                smallSolutionFoundDFSR = true;
                System.out.println(dfsRecursiveNodesVisited);
                while (!dfsStack.empty()) {
                    System.out.print(dfsStack.pop() + " ");
                }
            } else {
                smallSolutionFoundDFSR = false;
            }
        }
    }

    /**
     * Prints a DFSIt solution if no testing is in place.
     *
     * @param dfsItStack Stack to print from
     */
    public void printDFSItSolution(Stack dfsItStack) {
        if (testing == false) {
            if (dfsIterativNodesVisited < 100) {
                System.out.println(dfsIterativNodesVisited);
                while (!dfsItStack.empty()) {
                    System.out.print(dfsItStack.pop() + " ");
                }
            }
        }

    }

    /**
     * Prints a BFS solution if no testing is in place.
     *
     * @param state State to print from
     */
    public void printBFSSolution(State state) {
        if (testing == false) {
            System.out.println(bfsNodesVisited);
            State printState = state;
            while (printState != null) {
                System.out.print(printState.agent + " ");
                printState = printState.previosState;
            }
        }
    }

    /**
     * Checks if all the current coordinates of our state are the same as the
     * coordinates on the final state.
     *
     * @return True is it is a solution
     */
    public boolean checkIfCurrentStateIsSolution() {
        for (int i = 0; i < finalCoord.size(); i++) {
            if (!currentCoord.get(i).equals(finalCoord.get(i))) {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks if all the current coordinates of a given state are the same as
     * the coordinates on the final state.
     *
     * @param state Checks this state
     * @return True is it is a solution
     */
    public boolean checkIfCurrentStateIsSolution(State state) {
        for (int i = 0; i < finalCoord.size(); i++) {
            if (!finalCoord.get(i).equals(state.blocks.get(i))) {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks if a coordinate is in bound and if there are no obstacles on it
     *
     * @param x Coord x
     * @param y Coord y
     * @return True if the coordinate is in bound and if not on an obstacle.
     */
    public boolean CoordinatesInBouds(int x, int y) {
        //Check obstacle
        if (obstacleCoord != null) {
            for (int i = 0; i < obstacleCoord.size(); i++) {
                Coordinates currentObstacle = obstacleCoord.get(i);
                if (currentObstacle.x == x && currentObstacle.y == y) {
                    return false;
                }
            }
        }
        //Check if it is on board
        if (x <= boardSizeX && x > 0 && y > 0 && y <= boardSizey) {
            return true;
        }
        return false;
    }

    /**
     * Checks if a coordinate object is in bound and if there are no obstacles
     * on it
     *
     * @param x Coordinate object
     * @return True if the coordinate is in bound and if not on an obstacle.
     */
    public boolean CoordinatesInBouds(Coordinates x) {
        return CoordinatesInBouds(x.x, x.y);
    }

    /**
     * Changes the position of an agent by moving any blocks from the new
     * position to the old position.
     *
     * @param oldPosition Old position of the agent
     * @param newPosition New position of the agent
     */
    public void changeBlockLocation(Coordinates oldPosition, Coordinates newPosition) {
        for (int i = 0; i < currentCoord.size(); i++) {
            if (newPosition.equals(currentCoord.get(i))) {
                currentCoord.set(i, oldPosition);
                return;
            }
        }
    }

    /**
     * Changes the position of an agent within a state by moving any blocks from
     * the new position to the old position.
     *
     * @param oldPosition Old position of the agent
     * @param newPosition New position of the agent
     * @param state State to work on
     */
    public void changeBlockLocation(Coordinates oldPosition, Coordinates newPosition, State state) {
        for (int i = 0; i < state.blocks.size(); i++) {
            if (newPosition.equals(state.blocks.get(i))) {
                state.blocks.set(i, oldPosition);
                return;
            }
        }
    }
}

/**
 * This class is used to store a representation of an object in a 2D
 * environment.
 *
 * @author Amarandei Stanescu Alexandru
 */
class Coordinates {

    public int x, y;

    Coordinates() {
        this(0, 0);
    }

    Coordinates(int x, int y) {
        this.x = x;
        this.y = y;
    }

    Coordinates(Coordinates x) {
        this(x.x, x.y);
    }

    /**
     * Copies the values to this object from object X
     *
     * @param x Object to copy values from
     */
    public void getValuesFrom(Coordinates x) {
        this.x = x.x;
        this.y = x.y;
    }

    /**
     * Return the Manhattan distance between 2 coordinates object.
     *
     * @param x First coordinate
     * @param y Second coordinate
     * @return The Manhattan distance
     */
    static int manhattanDistance(Coordinates x, Coordinates y) {
        int xdist = x.x - y.x;
        int ydist = x.y - y.y;
        if (xdist < 0) {
            xdist = -xdist;
        }
        if (ydist < 0) {
            ydist = -ydist;
        }
        return xdist + ydist;
    }

    /**
     * Simple comparator function
     *
     * @param x Object to compare to
     * @return True if both x and y are equal
     */
    public boolean equals(Coordinates x) {
        if (this.x == x.x && this.y == x.y) {
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "(" + Integer.toString(x) + ", " + Integer.toString(y) + ") ";
    }
}

/**
 * This function represents a state that stores the current position of blocks,
 * it's predecessor and the position of the agent.
 *
 * @author Amarandei Stanescu Alexandru
 */
class State {

    public Coordinates agent;
    public ArrayList<Coordinates> blocks;
    public State previosState = null;

    public State(ArrayList blocks, Coordinates agent) {
        this.blocks = blocks;
        this.agent = agent;
    }

    public void putValues(ArrayList blocks, Coordinates agent) {
        this.blocks = blocks;
        this.agent = agent;
    }

    @Override
    public String toString() {
        return blocks.toString() + " " + agent.toString();
    }

    /**
     * Simple equals function that compares this state to another state
     *
     * @param state State to compare to
     * @return True if the state has all the components equal to the given
     * state.
     */
    public boolean equals(State state) {
        if (state == null) {
            return false;
        }
        if (!agent.equals(state.agent)) {
            return false;
        }
        if (blocks.size() != state.blocks.size()) {
            return false;
        }
        for (int i = 0; i < blocks.size(); i++) {
            if (!blocks.get(i).equals(state.blocks.get(i))) {
                return false;
            }
        }
        return true;
    }
}

/**
 * This class is an extension of the normal state used in ASTar to store the
 * value and number of steps.
 *
 * @author Amarandei Stanescu Alexandru
 */
class ASState extends State implements Comparable<ASState> {

    public int steps;
    public int value;

    public ASState(ArrayList blocks, Coordinates agent) {
        super(blocks, agent);
        steps = 0;
        value = 0;
    }

    /**
     * Used for priority queue
     *
     * @param toComapareTo
     * @return
     */
    @Override
    public int compareTo(ASState toComapareTo) {
        if (toComapareTo.value < value) {
            return 1;
        }
        if (toComapareTo.value == value) {
            return 0;
        }
        return -1;
    }

}

/**
 * This class is used to represent a node for a dfs. I decided to make this
 * class in order to have a way to save the path of the solution and to minimize
 * the amount of data on the stack. Each node has it's own shuffeled movement
 * and can return an unexplored node.
 *
 * @author Amarandei Stanescu Alexandru
 */
class DfsNode {

    public Coordinates Coordinates, predecesor;
    public ArrayList<Integer> shufelledMovement;
    public int currentNode;

    DfsNode(Coordinates Coordinates) {
        currentNode = 0;
        shufelledMovement = new ArrayList<>();
        shufelledMovement.add(0);
        shufelledMovement.add(1);
        shufelledMovement.add(2);
        shufelledMovement.add(3);
        this.Coordinates = new Coordinates();
        this.Coordinates.x = Coordinates.x;
        this.Coordinates.y = Coordinates.y;
        Collections.shuffle(shufelledMovement);
    }

    /**
     * This function returns the next unexplored node.
     *
     * @return next possible node
     */
    public DfsNode nextDFSNode() {
        int currentMove = shufelledMovement.get(currentNode);
        currentNode++;
        DfsNode newNode;
        Coordinates newCoordinates = null;
        //See what is the next node that should be returned and assign it's values
        if (currentMove == 0) {
            newCoordinates = new Coordinates(Coordinates.x + 1, Coordinates.y);
        }
        if (currentMove == 1) {
            newCoordinates = new Coordinates(Coordinates.x, Coordinates.y - 1);
        }
        if (currentMove == 2) {
            newCoordinates = new Coordinates(Coordinates.x - 1, Coordinates.y);
        }
        if (currentMove == 3) {
            newCoordinates = new Coordinates(Coordinates.x, Coordinates.y + 1);
        }
        //Then we add the coordonates to the node
        newNode = new DfsNode(newCoordinates);
        newNode.predecesor = new Coordinates(Coordinates.x, Coordinates.y);

        return newNode;
    }
}
