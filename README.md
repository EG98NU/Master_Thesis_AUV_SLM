# Master Thesis: Managing AUV missions through small language models and behavior trees

This repository contains the code that I wrote to deploy the simulations necessary to develop the thesis, which you will find here too and it explains the code purpose.

## PoC
This directory contains the code developed for the Proof of Concept section of the thesis.

Brief description of each script:
* chat: allows the user to write commands to the system
* environment: contains the mission objects and features of the mission, written in the form defined by the "state.py" classes
* executor: executes the functions that update the state machine
* operations: blocks of functions that the planner calls
* state: defines the state machine
* system: receives the commands and uses the planner to call the operations and execute them
* t5base_planner: the SLM planner

### Usage
Run in a terminal

`cd PoC`

`python3 chat.py`

In another terminal run

`cd PoC`

`python3 system.py`

Write the command on the chat terminal and send, monitor the mission development on the system terminal.

### Dataset
The dataset built to train the model can be obtained running

`cd PoC`

`python3 build_dataset_PoC.py`

## tlm2ros

This directory contains the code used for the application of the approach developed in the thesis.

This system connects to the ROS environment where the AUV control system is deployed, which I cannot disclose in this repository since it was developed by the ERGO team at University of Pisa and it is private.

Nevertheless the code I wrote personally for the purpose in the "mm_bt" directory contains the bridge to the manager in "tlm2ros" and the elements necessary to pursue the simulation on ROS through Stage.

  ### SLM environment
  Brief description of elements:
  * chat: allows the user to write commands
  * planner: uses the SLM to generate the missions to send to the ROS env.
  * manager: This is the SLM manager. receives info from the ROS env., the command from the chat and uses the planner, then sends the mission to the bridge
  * ros_connection: communicates with the bridge

  ### Dataset
  The dataset used to train the model can be obtained running on a SLM env. terminal

  `cd tlm2ros`
  
  `python3 build_dataset.py`

  # Training
  Using the dataset building scripts in PoC or tlm2ros directories will provide a training and a testing dataset.

  "fine_tune_flant5base.ipynb" is a notebook that was used on Google Colab with T4 runtime to fine tune FLAN-T5-base using the training dataset.
  "test_flant5base.ipybn" is a notebook that tests the fine tuned model with the testing dataset and computes confidence statistics.

  ## Usage
  To use the fine tune notebook make sure to upload the notebook and the training dataset in the same directory on Google Drive and adjust the path names at the     top of the notebook and run all cells, the notebook will fine tune the model and save it and provide training statistics.

  To use the test notebook make sure to have in the same directory the notebook, the model to test and the testing datasets. Adjust the path names and execute all   cells. The notebook will provide test statistics.
  
