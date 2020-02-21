/**********************************************
* Function file for pacman AI.                *
* Written by Tom Hanson 913022 - (2019)       *
* with code addapted from Nir Lipovetzky      *
***********************************************/

#include <time.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <assert.h>

#include "ai.h"
#include "utils.h"
#include "priority_queue.h"

//Definitions
#define INVALID_MOVE -100
#define DISCOUNT_VALUE 0.98
#define NUM_MOVES 4
#define INVINC_VAL 20
#define LOST_LIFE_VAL 20
#define GAME_OVER_VAL 100
#define ERROR -1



/*****************************************************************
* Function:    get_next_move                                     *
* Parameters:  state_t initial state, int budget, propagation_t  *
*              propagation type, char* stats, move_t previous,   *
*              move_t pre previous, output_t* output             *
* Returns:     move_t best move                                  *
* Description: Find best action by building all possible paths   *
*              up to a budget and finds the movement with the    * 
*              highest score using either max or avgerage.       * 
*****************************************************************/
move_t get_next_move(state_t init_state, int budget, propagation_t propagation,
                     char* stats,move_t prev,move_t p_prev,output_t* output){

//Define and initialise variables
    move_t best_action = 0;
    node_t* current_node = NULL;
    node_t* explored[budget];
    node_t* first_moves[NUM_MOVES] = {NULL};
    float best_action_score[NUM_MOVES] = {INVALID_MOVE};
    int i = 0;
    int j = 0;
    int first_loop = 1;
    int expl = 0;
    
    node_t* *new_node = (node_t**) malloc(sizeof(node_t*));
    assert(new_node);
    
    for(i = 0; i < budget;i++){
        explored[i] = NULL;
    }
    
    output->prop = propagation;
    output->budget = budget;
	
    //Add the initial node
    node_t* init = create_init_node( &init_state, prev);
    assert(init);
    output->generated += 1;
	heap_push(h,init);
    
    //Generate and explore game states until budget reached
	while((expl<budget)&&(h->heaparr[0] != NULL)){

        //Pop out previous node and add it to explored array        
        current_node = heap_delete(h);
        explored[expl] = current_node;
        expl++;
        output->expanded += 1;
        
        //For all 4 movement opperations explore the outcomes
        for(i = 0; i < NUM_MOVES; i++){
        
            exploreAction(current_node, new_node, explored, &expl,
                          propagation, budget, i, p_prev, output);
            
            //If it is one of the first movements, add it to the first movement array
            if(first_loop){
                first_moves[j] = *new_node;
                j++; 
            }
        }
        
        first_loop = 0;
    }
    


    //Find the best action score, choosing between equal scores randomly    
    best_action = getBestAction(first_moves,best_action_score);
    
    if(best_action_score[best_action] > output->max_board){
        output->max_board = best_action_score[best_action];
    }
    
    //Print out statistics           
	sprintf(stats, "Max Depth: %d Expanded nodes: %d  Generated nodes: %d\n",output->max_depth,output->expanded,output->generated);
	
	if(best_action == left)
		sprintf(stats, "%sSelected action: Left\n",stats);
	if(best_action == right)
		sprintf(stats, "%sSelected action: Right\n",stats);
	if(best_action == up)
		sprintf(stats, "%sSelected action: Up\n",stats);
	if(best_action == down)
		sprintf(stats, "%sSelected action: Down\n",stats);

	sprintf(stats, "%sScore Left %.6f Right %.6f Up %.6f Down %.6f",stats,best_action_score[left],best_action_score[right],best_action_score[up],best_action_score[down]);
    
    //Free memory and the heap
    max_heapify(h->heaparr, 0, h->count);
    while(h->heaparr[0] != NULL){
        free(heap_delete(h));
    }
    
    for(i = 0; i < budget ;i++){
        if(explored[i] != NULL){
        free(explored[i]);
        }
    }
    
    free(new_node);
	return best_action;
}

/*****************************************************************
* Function:    exploreAction                                     *
* Parameters:  node_t* current node, node_t** new node, node_t** *
*              explored array,int* expl, propagation_t prop,     *
*              int budget, move_t next action, move_t p_previous *
*              output_t* output                                  *
* Returns:     none                                              *
* Description: Explores the next state based on the desired      *
*              action. Then propagates the score back to the     * 
*              movement node                                     * 
*****************************************************************/
void exploreAction(node_t* current_node,node_t** new_node,node_t** explored, int* expl,propagation_t propagation,int budget,move_t action,move_t p_prev, output_t* output){
    
    //Initialise pointers     
    *new_node = NULL;
    node_t* opp_node = (node_t *) malloc(sizeof(node_t));
    assert(opp_node);
    opp_node->parent = current_node;
    
    
    //Check if node is under budget and that the direction of movement is allowed    
    if((*expl+1<budget) && checkDir(current_node->state,action)){
        
        //Simulate state resulting from opposite action        
        testAction(current_node,opp_node,getOpposite(action));

        //Check if pacman backtracts, and if he does, checks if not backtracking 
        //results in a lost life
        if(checkBacktrack(current_node, action)&& !lostLife(opp_node) ){
            free(opp_node);
            return;
        }else{
            
            // apply the desired action to the node
            applyAction(current_node,new_node,action);
            output->generated += 1;
        
            //propagates the score of the new state back to first ancestor
            propagateBackScoreToFirstAction(*new_node,propagation);
        
            //If new state results in a loss of life, dont add it to the heap
            if(lostLife(*new_node) ){
                
                free(*new_node);
                free(opp_node);
                return;
            }
            
            //otherwise add it to the heap
            heap_push(h,*new_node);
            
            //Update auxiliary data and free opp_node
            if((*new_node)->depth > output->max_depth){
                output->max_depth = (*new_node)->depth;
            }
            
            free(opp_node);
            return;
        }
    }else{
        free(opp_node);
        return;
    }
}

/****************************************************************
* Function:    getBestAction                                    *
* Parameters:  node_t* first moves[], float best action score[] *                 
* Returns:     move_t best action                               *
* Description: Gets the best action scores from each of the     *
*              first movement nodes an returns the highest      *
*              scoring node                                     *
****************************************************************/
move_t getBestAction(node_t* first_moves[],float best_action_score[]){
    
    //Initialise variables
    int i = 0;
    move_t best_action = 0;
    
    //If the move is valid, add its score to the best score array
    for(i = 0;i < NUM_MOVES; i++){
        if(first_moves[i] != NULL){
                best_action_score[i] = first_moves[i]->acc_reward;      
        }else{
            best_action_score[i] = INVALID_MOVE;
        }
    }
    
    //Find the best score from the best_action array, if 2 scores
    //are the same, choose randomly
    for(i = 0; i < NUM_MOVES; i++){
        if(best_action_score[best_action] < best_action_score[i]){
            best_action = i;
        }else if((best_action_score[best_action] == best_action_score[i])&&(rand()%2)){
            best_action = i;
        }
    }

    //The best action is returned
    return best_action;
}


/****************************************************************
* Function:    testAction                                       *
* Parameters:  node_t* current node, node_t* new node, move_t   *
*              action                                           *                 
* Returns:     none                                             *
* Description: Apply an action to node n and return a new node  *
*              resulting from executing the action              *
****************************************************************/
void testAction(node_t* n, node_t* new_node, move_t action ){
    
    //The current state is copied to the new node and the 
    //new state simulated
    copy_state(&(new_node->state), &(n->state));
    execute_move_t( &(new_node->state), action );
}

/****************************************************************
* Function:    applyAction                                      *
* Parameters:  node_t* current node, node_t* new node, move_t   *
*              action                                           *                 
* Returns:     bool changed direction                           *
* Description: Apply an action to node n and return a new node  *
*              resulting from executing the action. Also updates*
*              auxiliary information associated with the new    *
*              node and its parents                             *
****************************************************************/
bool applyAction(node_t* n, node_t** new_node, move_t action ){

	bool changed_dir = false;
    
    //Allocating memory for new node
    *new_node = (node_t *) malloc(sizeof(node_t));
    assert(*new_node);
    
    //Copying the current state to the new node and the 
    //new state is simulated
    copy_state(&((*new_node)->state), &(n->state));
    execute_move_t( &((*new_node)->state), action );

    node_t* current = *new_node;

    //Updating the new_node metadata 
    (*new_node)->parent = n;
    (*new_node)->move = action;
    (*new_node)->depth = n->depth + 1;
	(*new_node)->priority = -((*new_node)->depth);
	(*new_node)->num_childs = 0;
	(*new_node)->acc_reward =  get_reward(*new_node);
    
    //Updating number the of children of all parent nodes
    while(current->parent != NULL){
        current->parent->num_childs += 1;
        current = current->parent;
    }
    
    //Testing if direction has changed
    if((n->move) != action){
        changed_dir = true;
    }
    
	return changed_dir;
}

/****************************************************************
* Function:    heuristic                                        *
* Parameters:  node_t* current node                             *                 
* Returns:     float heuristic value                            *
* Description: Calcuates the heuristic value of a game state    *
****************************************************************/
float heuristic( node_t* n ){
	float h = 0;
    
    //Calculates a heuristic value of the state 
    if(n->parent != NULL){
        //Tests if pacman is now invincible
        float i = INVINC_VAL*(n->state.Invincible > n->parent->state.Invincible);
        
        //Tests if pacman has lost a life
        float l = LOST_LIFE_VAL*lostLife(n);
        
        //Tests if the game is over
        float g = GAME_OVER_VAL*(n->state.Lives == -1);
        h = i - l - g;
    }
	

	return h;
}

/****************************************************************
* Function:    get_reward                                       *
* Parameters:  node_t* current node                             *                 
* Returns:     float reward value                               *
* Description: Calcuates the reward value of a game state       *
****************************************************************/
float get_reward ( node_t* n ){
	float reward = 0;
	float points = 0;
    
    //Gets the net gain of points from previous to current state 
    if(n->parent != NULL){
        points = n->state.Points - n->parent->state.Points;
    }else{
        points = n->state.Points;
    }
    
    //Calculates the total reward based on the points and the 
    //heuristic value of the state
	reward = heuristic(n) + points;

    //discounts the reward based on how far into the future 
    //the state is
	float discount = pow(DISCOUNT_VALUE,n->depth);
   	
	return discount * reward;
}

/****************************************************************
* Function:    getOpposite                                      *
* Parameters:  move_t action                                    *                 
* Returns:     move_t opposite action                           *
* Description: Finds the action in the opposite direction to    *
*              input action                                     *
****************************************************************/
move_t getOpposite(move_t action){
    
    //Returns the move_t in the opposite direction of action
    switch(action){
        case left:
            return right;
            break;
        case right:
            return left;
            break;
        case up:
            return down;
            break;
        case down:
            return up;
            break;
    }
    return ERROR;
}

/****************************************************************
* Function:    checkBacktrack                                   *
* Parameters:  node_t* current node, move_t previous,           *
*              move_t action                                    *                 
* Returns:     bool backtrack                                   *
* Description: Returns true if the last move made is the        *
*              opposite to the current move                     *
*              input action                                     *
****************************************************************/
bool checkBacktrack(node_t* node,move_t action){
    
    //Gets the opposite action
    move_t opposite = getOpposite(action);
    
    //Returns true if the move of the previous node is 
    //opposite to the next action 
    if(node->move == opposite){
        return true;
    }else{
        return false;
    }   
}
             

/****************************************************************
* Function:    copy_state                                       *
* Parameters:  state_t* destination, state_t* source            *              
* Returns:     none                                             *
* Description: Copies state information from source state to    *
*              destination state                                *
****************************************************************/
void copy_state(state_t* dst, state_t* src){
	//Location of Ghosts and Pacman
	memcpy( dst->Loc, src->Loc, 5*2*sizeof(int) );

    //Direction of Ghosts and Pacman
	memcpy( dst->Dir, src->Dir, 5*2*sizeof(int) );

    //Default location in case Pacman/Ghosts die
	memcpy( dst->StartingPoints, src->StartingPoints, 5*2*sizeof(int) );

    //Check for invincibility
    dst->Invincible = src->Invincible;
    
    //Number of pellets left in level
    dst->Food = src->Food;
    
    //Main level array
	memcpy( dst->Level, src->Level, 29*28*sizeof(int) );

    //What level number are we on?
    dst->LevelNumber = src->LevelNumber;
    
    //Keep track of how many points to give for eating ghosts
    dst->GhostsInARow = src->GhostsInARow;

    //How long left for invincibility
    dst->tleft = src->tleft;

    //Initial points
    dst->Points = src->Points;

    //Remiaining Lives
    dst->Lives = src->Lives;   

}

/****************************************************************
* Function:    initialize_ai                                    *
* Parameters:  none                                             *              
* Returns:     struct heap*                                     *
* Description: Initialises heap structure                       *
****************************************************************/
struct heap* initialize_ai(void){
    
    //Allocates memory and initialises heap
    struct heap* h = (struct heap*) malloc(sizeof(struct heap));
        assert(h);
	heap_init(h);
    return h;
}

/****************************************************************
* Function:    create_init_node                                 *
* Parameters:  state_t* initial state, move_t previous move     *                           
* Returns:     node_t* initial node                             *
* Description: Creates initial node based on initial state info *
****************************************************************/
node_t* create_init_node( state_t* init_state, move_t prev ){
	
    //Allocates memory for new node
    node_t * new_n = (node_t *) malloc(sizeof(node_t));
    
    //Updates node's metadata
	new_n->parent = NULL;	
	new_n->priority = 0;
    new_n->move = prev;
	new_n->depth = 0;
	new_n->num_childs = 0;
	copy_state(&(new_n->state), init_state);
	new_n->acc_reward =  get_reward( new_n );
	return new_n;
	
}

/****************************************************************
* Function:    propagateBackScoreToFirstAction                  *
* Parameters:  node_t* new node,propagation_t propagation       *                           
* Returns:     none                                             *
* Description: Propagates the current nodes' score to its first *
*              ancestor using either a maximum or average       *
*              propagation type                                 *
****************************************************************/
void propagateBackScoreToFirstAction(node_t* new_node,propagation_t propagation){
    node_t* current = new_node;
    float score = 0;
    
    //Chooses propatation method according to propagation
    switch(propagation){
        case max:
            
            //Adds the score from all nodes until first ancestor
            while(current->parent->depth != 0){
                score += current->acc_reward;
                current = current->parent;
            }
            
            //If total score is greater than previous largest,
            //update first ancestor's acc_reward to new score
            if(score >  current->acc_reward){
                current->acc_reward = score;
            }
            
            break;
            
        case avg:
            
            //Gets score from current node
            score = current->acc_reward;
            
            //Traverses to first ancestor
            while(current->parent->depth != 0){
                current = current->parent;
            }
            
            //Adds new score to the total average
            current->acc_reward +=  (score - current->acc_reward)/(current->num_childs + 1);
            break;
    }
    
}


/****************************************************************
* Function:    checkDir                                         *
* Parameters:  state_t current state, move_t next move          *                           
* Returns:     bool valid move                                  *
* Description: Returns true if the next move is valid in the    *
*              current state                                    *
****************************************************************/
bool checkDir(state_t state,move_t move){
    int location[2];
    
    //Finds the location of pacman
    location[0] = state.Loc[4][0];
    location[1] = state.Loc[4][1];
    
    //Finds pacmans new location if he were to move in the 
    //desired direction
    switch (move) {
    case up:          
        location[0] += -1;
        location[1] += 0; 
        break;

    case down:         //Move pacman down
        location[0] += 1;
        location[1] += 0; 
        break;

    case left:         //Move pacman left
        location[0] += 0;
        location[1] += -1; 
        break;


    case right:          //Move pacman right
        location[0] += 0;
        location[1] += 1; 
        break;
    }

        //If he would hit a wall, return false 
        if((Level[location[0]][location[1]] == 1) || (Level[location[0]][location[1]] == 4)) {
            return false;
        }else{
            return true;
        }
    }

/****************************************************************
* Function:    lostLife                                         *
* Parameters:  node_t* n                                        *                           
* Returns:     bool Lost life                                   *
* Description: Returns true if the next move looses pacman a    *
*              life                                             *
****************************************************************/
bool lostLife(node_t* n){
    
    //Checks if the parent exists, if it doesnt, return false 
    if(n->parent == NULL){
        return false;
    }else{
        
        //If the parent exists, check if the number of lives of current
        //state is less than previous state, if so return true
        if(n->state.Lives < n->parent->state.Lives){
            return true;
        }else{
            return false;
        }
    }
    
}

