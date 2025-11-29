/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @author Skanda Nagendra 101299202
 * @author Aaron Fisher 101294988
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */
#include <map>
#include "interrupts_101299202_101294988.hpp"
#define TIME_QUANTUM 100


void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}


std::tuple<std::string /* add std::string for bonus mark */ > run_simulation(std::vector<PCB> list_processes) {

    // I have added tracking variables for calculating all 4 metrics to analyse later
    
    //these are vectors to hold the calculated turnaround/wait/response times of all processes
    //will be used to calculate averages at the end
    std::vector<unsigned int> turnaround_times;
    std::vector<unsigned int> waiting_times;
    std::vector<unsigned int> response_times;
    
    //each process's wait time, time of entering ready state, and first run time 
    //are mapped to their PID, whcih can be added to the vectors above, 
    std::map<int, unsigned int> process_wait_time;
    std::map<int, unsigned int> enter_ready_state_time;
    std::map<int, unsigned int> first_run_time;


    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    unsigned int current_time = 0;
    unsigned int quantum_remaining = TIME_QUANTUM;
    unsigned int cpu_time_since_last_io = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    //Loop while till there are no ready or waiting processes.
    //This is the main reason I have job_list, you don't have to use it.
    while(!all_process_terminated(job_list) || job_list.empty()) {

        //Inside this loop, there are three things you must do:
        // 1) Populate the ready queue with processes as they arrive
        // 2) Manage the wait queue
        // 3) Schedule processes from the ready queue

        //Population of ready queue is given to you as an example.
        //Go through the list of proceeses
        for(auto &process : list_processes) {
            if(process.arrival_time == current_time) {//check if the AT = current time
                //if so, assign memory and put the process into the ready queue
                if (assign_memory(process)) {

                    process.state = READY; 
                    enter_ready_state_time[process.PID] = current_time; // record the when process enters READY
                    ready_queue.push_back(process); //Add the process to the ready queue
                    job_list.push_back(process); //Add it to the list of processes

                    execution_status += print_exec_status(current_time, process.PID, NEW, READY);
                }
                else {
                    process.state = NEW; // process remains in NEW
                    job_list.push_back(process); // will need to check again later. add it to job list again for nwo
                    std::cout << "Process " << process.PID << " could not be assigned memory at time " << current_time << std::endl;
                    
                }
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////
        //This mainly involves keeping track of how long a process must remain in the ready queue
         for (auto iter = wait_queue.begin(); iter != wait_queue.end();) {
            iter->io_time_left--;
            if (iter->io_time_left == 0) {
                iter->state = READY;
                enter_ready_state_time[iter->PID] = current_time; // another point where process enters READY, must be recorded
                ready_queue.push_back(*iter);
                sync_queue(job_list, *iter);
                
                execution_status += print_exec_status(current_time, iter->PID, WAITING, READY);
                iter = wait_queue.erase(iter);
            }
            else {
                ++iter;
            }
        }
        /////////////////////////////////////////////////////////////////

        //////////////////////////SCHEDULER//////////////////////////////
        //Checking for running process
        if (running.state == RUNNING){
            running.remaining_time--;
            quantum_remaining--;
            cpu_time_since_last_io++;

            //check if the process completed
            if (running.remaining_time == 0){
                execution_status += print_exec_status(current_time, running.PID, running.state, TERMINATED);
                turnaround_times.push_back(current_time - running.arrival_time); // TT = completion time - arrival time
                waiting_times.push_back(process_wait_time[running.PID]); // wait time is added to teh waiting vector right before ending the process
                terminate_process(running, job_list);
                idle_CPU(running);
                quantum_remaining = TIME_QUANTUM;
                cpu_time_since_last_io = 0;
            }
            
            //check if the process needs to do I/O
            else if(running.io_freq > 0 && cpu_time_since_last_io >= running.io_freq) {
                // move to wait queue
                running.state = WAITING;
                running.io_time_left = running.io_duration;
                wait_queue.push_back(running);
                sync_queue(job_list, running);

                execution_status += print_exec_status(current_time, running.PID, RUNNING, WAITING);
                idle_CPU(running);
                quantum_remaining = TIME_QUANTUM;
                cpu_time_since_last_io = 0;
            }

            // check if the time queue expired (RR will preempt)
            else if (quantum_remaining == 0){
                running.state = READY;
                enter_ready_state_time[running.PID] = current_time; // process enters/re-enters ready state, must be recorded
                ready_queue.insert(ready_queue.begin(), running);
                sync_queue(job_list, running);

                execution_status += print_exec_status(current_time, running.PID, RUNNING, READY);
                idle_CPU(running);
                cpu_time_since_last_io = 0;
            }
        }

        

        /////////////////////////////////////////////////////////////////

        //sSchedule a new process if CPU is idle
        if(running.state != RUNNING && !ready_queue.empty()) {
            running = ready_queue.back();
            ready_queue.pop_back();

            // only if the process is starting for the first time will the start time be set to the current time
            // otherwise it retains its original start time
            // This is to prevent overwriting the start time when a process is preempted and rescheduled
            // run_process() function sets start time to current time unconditionally, meaning every time a process is scheduled
            // its start time would be updated (even if it is just being rescheduled after preemption), which is incorrect.
            if (running.start_time == -1) {
                running.start_time = current_time;
            }

            states old_state = running.state;
            running.state = RUNNING;
            quantum_remaining = TIME_QUANTUM;
            sync_queue(job_list, running);
            execution_status += print_exec_status(current_time, running.PID, old_state, running.state);

            //if this is the first time the process is running, record the response time
            if (first_run_time.find(running.PID) == first_run_time.end()) {
                first_run_time[running.PID] = running.start_time;
                response_times.push_back(current_time - running.arrival_time); // RT = first run time - arrival time
            }

            //calculate the waiting time up to this point
            //checks if the process has entered the ready state before. if so, calculate the wait time  
            if(enter_ready_state_time.find(running.PID) != enter_ready_state_time.end()) {
                //initializes a wait counter to 0 only if this is the first time process is entering WAIT state
                if (process_wait_time.find(running.PID) == process_wait_time.end()) {
                    process_wait_time[running.PID] = 0;
                }
                process_wait_time[running.PID] += (current_time - enter_ready_state_time[running.PID]);
            }
        }
        
        current_time++;
    }   
    
    //Close the output table
    execution_status += print_exec_footer();

    //time to calculate all the averages yay
    float avg_turnaround_time = 0;
    float avg_waiting_time = 0; 
    float avg_response_time = 0;
    float throughput = 0;

    for (auto tt : turnaround_times) {
        avg_turnaround_time += tt;
    }
    avg_turnaround_time = avg_turnaround_time / turnaround_times.size(); 

    for (auto wt : waiting_times) {
        avg_waiting_time += wt;
    }
    avg_waiting_time = avg_waiting_time / waiting_times.size();

    for (auto rt : response_times) {
        avg_response_time += rt;
    }
    avg_response_time = avg_response_time / response_times.size(); 

    throughput = (float) turnaround_times.size() / (float) current_time; // total number of processes / total time taken. could also use job_list.size() i think
    throughput = throughput*1000; //change to proecess/second
   
    std::cout << "\n=== METRICS ===" << std::endl;
    std::cout << "Throughput: " << throughput << " processes/sec" << std::endl;
    std::cout << "Avg Turnaround Time: " << avg_turnaround_time << " ms" << std::endl;
    std::cout << "Avg Wait Time: " << avg_waiting_time << " ms" << std::endl;
    std::cout << "Avg Response Time: " << avg_response_time << " ms" << std::endl;

    return std::make_tuple(execution_status);
}

int main(int argc, char** argv) {

    //Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrutps <your_input_file.txt>" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "output_files/RR/execution_RR.txt");

    return 0;
}