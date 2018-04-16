/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

//Macros for easier coding
#define SRC1(stage) mips_state.pipeStageState[stage].cmd.src1
#define SRC2(stage) mips_state.pipeStageState[stage].cmd.src2
#define DST(stage) mips_state.pipeStageState[stage].cmd.dst
#define SRC1VAL(stage) mips_state.pipeStageState[stage].src1Val
#define SRC2VAL(stage) mips_state.pipeStageState[stage].src2Val
#define OPCODE(stage) mips_state.pipeStageState[stage].cmd.opcode
#define ISSRC2IMM(stage) mips_state.pipeStageState[stage].cmd.isSrc2Imm
#define GPR(number) mips_state.regFile[number]

//Declarations of auxiliary functions
void pass_command_from_to(pipeStage from_stage, pipeStage to_stage);
void insert_nop_to_stage(pipeStage stage);
bool detect_data_hazard();
bool detect_load_hazard();
void run_alu();
void branch_taken();
void update_decode();

//A class to hold more information needed for MIPS opeartion
class info {
public:
	//EXE/MEM latches
	int alu_result; //Holds the result of the alu action (exe/mem latch )
	int address_for_jump; //Holds info of the address we need to jump to

	//MEM/WB latches
	int mem_wb_latch;	//Holds the info of the alu action, after one cycle
	int writeback_wire; //Holds info that needs to be written in WB
	bool branch_resolution;	//Holds the resolution of the branch from stage MEM,
							//so we can use it in WB stage.

	//WB wires
	int previous_wb_register; //Holds the number of the register last used in WB
	int waiting_to_write;

	//General variables
	bool memory_busy;		 //True if we still read from memory last cycle

	//For forwarding
	int mem_forwarding;		//A wire for forwarding from MEM to EXE
	int wb_forwarding; 		//A wire for forwarding from WB to EXE
	int updated_dst_value; //A wire for forwarding to BRANCH commands
};

//A static variable representing the entire state of our machine
static SIM_coreState mips_state;

//A static variable used to save more information needed
static info extra_info;

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instruction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {

	//Setting the PC to be 0 as required
	mips_state.pc = 0;
	
	//Resetting all the registers to 0
	for(int i = 0; i < SIM_REGFILE_SIZE ; i++) {
		GPR(i) = 0;
	}

	//Resetting the pipeline stages
	for(int i = 1 ; i < SIM_PIPELINE_DEPTH ; i++) {
		OPCODE(i) = CMD_NOP;
	}

	//Loading the instruction of address 0x0 into the pipe
	SIM_MemInstRead(0,&mips_state.pipeStageState[FETCH].cmd);

	return 0;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {

	/*************************************************************************/
	/* WB - Write back stage. We need to write the information to the 		 */
	/*  	register file. Notice that if split regfile is not activated     */
	/*		then we only write at the end of the cycle						 */
	/*************************************************************************/

	//If split regfile is not activated, then we write to memory what the last
	//command wanted. By doing so - we are not allowing ID stage to read the
	//information it needs in the previous cycle, because it wasn't written yet.
	extra_info.previous_wb_register = 0 ;
	if(split_regfile == false) {
		if((OPCODE(WRITEBACK) == CMD_LOAD || OPCODE(WRITEBACK) == CMD_ADD || OPCODE(WRITEBACK) == CMD_SUB)
				&& DST(WRITEBACK) != 0) {
			GPR(DST(WRITEBACK)) = extra_info.waiting_to_write;
			extra_info.wb_forwarding = extra_info.waiting_to_write;
			extra_info.previous_wb_register = DST(WRITEBACK);
		}
	}

	//If the branch resolution was true last cycle, then when the clock ticks
	//we have to load our new PC and bubble out the other commands. This part is
	//independent from "split_regfile". It happens always.
	if(extra_info.branch_resolution == true) {
		pass_command_from_to(MEMORY,WRITEBACK);
		branch_taken();
		return;
	}

	//If we haven't finished reading the memory last cycle then we don't care
	//about whatever is in WB stage yet. Therefore we clean the pipe.
	if(extra_info.memory_busy == true) {
		insert_nop_to_stage(WRITEBACK);
	} else {
		if(split_regfile == true) {
			pass_command_from_to(MEMORY,WRITEBACK);
		//As soon as we pass the command we handle it, so values will be correct
		//when ID stage reads them.
			switch(OPCODE(WRITEBACK)) {
			case CMD_ADD:
			case CMD_SUB:
			case CMD_LOAD:
				//Write the information that needs to be written back to register file
				//Update the 'forwarding' unit from writeback
				if(DST(WRITEBACK) != 0 ){
					GPR(DST(WRITEBACK)) = extra_info.writeback_wire;
					extra_info.wb_forwarding = extra_info.writeback_wire;
				}
				break;
			default:
				break;
			}
		} else {
			//If split regfile is not activated, all we have left to do is to
			//move the commands down the pipe
			pass_command_from_to(MEMORY,WRITEBACK);
			extra_info.waiting_to_write = extra_info.writeback_wire;
		}
	}

	/*************************************************************************/
	/* MEM - Memory stage. We move the command and handle it according to 	 */
	/*		 opcode. Some commands end here(like branches) and some need more*/
	/* 		 work in the next stage.										 */
	/*************************************************************************/

	if(extra_info.memory_busy == true) {
		//If memory didn't finish reading last cycle, check if it is finished
		if(SIM_MemDataRead(extra_info.mem_wb_latch,&extra_info.writeback_wire) == -1 ) {
			//Reading is not finished yet. Wait another cycle by stalling the
			//the stages that occur after "Memory".
			update_decode();
			return;
		} else {
			//Reading is definitely finished, so we can continue.
			extra_info.memory_busy = false;
			update_decode();
			return;
		}
	} else {
		//Memory is not busy, so we can continue handling the command
		pass_command_from_to(EXECUTE,MEMORY);
		switch (OPCODE(MEMORY)) {
			case CMD_ADD:
			case CMD_SUB:
				extra_info.writeback_wire = extra_info.alu_result;
				extra_info.mem_forwarding = extra_info.alu_result;
				break;
			case CMD_STORE:
				SIM_MemDataWrite(extra_info.alu_result, SRC1VAL(MEMORY));
				extra_info.mem_forwarding = extra_info.alu_result;
				break;
			case CMD_LOAD:
				//Here we try to read. If it takes more then one cycle, we save
				//the address we read from so it won't be run over.
				 if(SIM_MemDataRead(extra_info.alu_result,&extra_info.writeback_wire) == -1 ) {
					 extra_info.memory_busy = true;
					 extra_info.mem_wb_latch = extra_info.alu_result;
				 }
				 break;
			case CMD_BR:
				extra_info.branch_resolution = true;
				break;
			case CMD_BREQ:
				if(extra_info.alu_result == 0) {
					extra_info.branch_resolution = true;
				}
				break;
			case CMD_BRNEQ:
				if(extra_info.alu_result != 0) {
					extra_info.branch_resolution = true;
				}
				break;
			default:
				break;
		}
	}

	/*************************************************************************/
	/* EXE - Execute stage. We move the command and handle it according to 	 */
	/*		 opcode. This is the point where we check for hazards and insert */
	/* 		 nops if we need them											 */
	/*************************************************************************/

	//Here we can always pass the command forward. In the worst case, when we
	//can't handle the command yet, we'll insert nop and revert this pass.
	pass_command_from_to(DECODE,EXECUTE);

	extra_info.updated_dst_value = GPR(DST(EXECUTE));

	if(detect_load_hazard() == true) {
		insert_nop_to_stage(EXECUTE);
		//By returning from here we actually made sure the instruction in "DECODE"
		//Did not pass to "EXE" - that means we inserted a bubble as needed
		update_decode();
		return;
	}

	if(detect_data_hazard() == true ) {
		if(forwarding == false) {
			insert_nop_to_stage(EXECUTE);
			update_decode();
			return;
		}
		//If forwarding is true then detect will handle what is needed.
	}

	//If we reached here we are guaranteed to have the correct values in each
	//register, so we can continue do what we need.
	run_alu();

	/*************************************************************************/
	/* ID - DECODE stage. We move the command and saving the values to 		 */
	/*		registers. Since we implemented a function for this, we use it.	 */
	/*************************************************************************/

	pass_command_from_to(FETCH,DECODE);
	update_decode();

	/*************************************************************************/
	/* IF - FETCH stage. Just to be on the safe side we clean the old data,  */
	/*		then we increment the PC and read the instruction.				 */
	/*************************************************************************/

	insert_nop_to_stage(FETCH);
	mips_state.pc += 4;
	SIM_MemInstRead(mips_state.pc,&mips_state.pipeStageState[FETCH].cmd);
}


/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
	//Updating the PC
	curState->pc = mips_state.pc;

	//Updating the entire register file
	for(int i = 0; i < SIM_REGFILE_SIZE ; i++) {
		curState->regFile[i] = GPR(i);
	}

	//Updating the stages
	for(int i = 0; i < SIM_PIPELINE_DEPTH ; i++) {
		curState->pipeStageState[i].cmd.opcode = OPCODE(i);
		curState->pipeStageState[i].cmd.src1 = SRC1(i);
		curState->pipeStageState[i].src1Val = SRC1VAL(i);
		curState->pipeStageState[i].cmd.src2 = SRC2(i);
		curState->pipeStageState[i].src2Val = SRC2VAL(i);
		curState->pipeStageState[i].cmd.isSrc2Imm = ISSRC2IMM(i);
		curState->pipeStageState[i].cmd.dst = DST(i);
	}

}

//=============================================================================
//=============================================================================
//Auxiliary functions


/**
 * An auxiliary function meant to pass commands along the pipe stages, by copying
 * the information about the command.
 */
void pass_command_from_to(pipeStage from_stage, pipeStage to_stage) {
	mips_state.pipeStageState[to_stage].cmd =
							mips_state.pipeStageState[from_stage].cmd;
	SRC1VAL(to_stage) = SRC1VAL(from_stage);
	SRC2VAL(to_stage) = SRC2VAL(from_stage);
}


/**
 * An auxiliary function meant to clear the pipe stages by inserting "Nop" command.
 * It is used both to "clear" a pipe stage after branch resolution and also for
 * stalling the pipe when needed.
 */
void insert_nop_to_stage(pipeStage stage) {
	OPCODE(stage) = CMD_NOP;
	SRC1(stage) = 0;
	SRC2(stage) = 0;
	ISSRC2IMM(stage) = false;
	DST(stage) = 0;
	SRC1VAL(stage) = 0;
	SRC2VAL(stage) = 0;
}


/**
 * An auxiliary function used for checking various cases of data hazard.
 * We check everything possible in order to know how to handle the hazard.
 * The classification to different cases helps us understand better what forwarding
 * action is needed (if possible).
 */
bool detect_data_hazard() {
	bool flag = false;
	//Note that we check "EXE" for data hazards between ID and MEM since this
	//function is only used after moving the command from ID to EXE

	//Case 1A  - Data hazard between ID and WB (Src1)
	if((SRC1(EXECUTE) == DST(WRITEBACK) || SRC1(EXECUTE) == extra_info.previous_wb_register) && SRC1(EXECUTE) != 0
			&& OPCODE(WRITEBACK) != CMD_STORE ) {
		if(forwarding == true) {
			SRC1VAL(EXECUTE) = extra_info.wb_forwarding;
		}
		flag = true;
	}

	//Case 1B  - Data hazard between ID and WB (Src2)
	if((SRC2(EXECUTE) == DST(WRITEBACK) || SRC2(EXECUTE) == extra_info.previous_wb_register) && SRC2(EXECUTE) != 0
			&& ISSRC2IMM(EXECUTE) == false && OPCODE(WRITEBACK) != CMD_STORE ) {
		if(forwarding == true) {
			SRC2VAL(EXECUTE) = extra_info.wb_forwarding;
		}
		flag = true;
	}

	//Case 2A  - if the command in ID is a branch or store and DST is not updated (wb)
	if((DST(EXECUTE) == DST(WRITEBACK) || DST(EXECUTE) == extra_info.previous_wb_register) && DST(EXECUTE) != 0 &&
			(OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ || OPCODE(EXECUTE) == CMD_BRNEQ || OPCODE(EXECUTE) == CMD_STORE)) {
		if(forwarding == true) {
			extra_info.updated_dst_value = extra_info.wb_forwarding;
		}
	    flag = true;
	}

	//Case 3A  - Data hazard between ID and MEM (Src1)
	if(SRC1(EXECUTE) == DST(MEMORY) && SRC1(EXECUTE) != 0 && OPCODE(EXECUTE) != CMD_BR
			&& OPCODE(EXECUTE) != CMD_NOP
			&& OPCODE(MEMORY) != CMD_BR && OPCODE(MEMORY) != CMD_BREQ && OPCODE(MEMORY) != CMD_BRNEQ
			&& OPCODE(MEMORY) != CMD_STORE ) {
		if(forwarding == true) {
			SRC1VAL(EXECUTE) = extra_info.mem_forwarding;
		}
		flag = true;
	}

	//Case 3B  - Data hazard between ID and MEM (Src2)
	if(SRC2(EXECUTE) == DST(MEMORY) && SRC2(EXECUTE) != 0 && OPCODE(EXECUTE) != CMD_BR
			&& OPCODE(EXECUTE) != CMD_NOP && ISSRC2IMM(EXECUTE) == false
			&& OPCODE(MEMORY) != CMD_BR && OPCODE(MEMORY) != CMD_BREQ && OPCODE(MEMORY) != CMD_BRNEQ
			&& OPCODE(MEMORY) != CMD_STORE ) {
		if(forwarding == true) {
			SRC2VAL(EXECUTE) = extra_info.mem_forwarding;
		}
		flag = true;
	}

	//Case 2B  - if the command in ID is a branch or store and DST is not updated (memory)
	if(DST(EXECUTE) == DST(MEMORY) && DST(EXECUTE) != 0 &&
			(OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ || OPCODE(EXECUTE) == CMD_BRNEQ || OPCODE(EXECUTE) == CMD_STORE)) {
		if(forwarding == true) {
			extra_info.updated_dst_value = extra_info.mem_forwarding;
		}
		 flag = true;
	}

	return flag;
}

/**
 * An auxiliary function that mimics the behavior of the ALU unit. We separate
 * between "regular" cases (i.e. SUB/ADD) and jumping cases (BR/BRNEQ/BREQ) so
 * we can save the information more easily and handle it better.
 */
void run_alu() {
	switch(OPCODE(EXECUTE)) {
	case CMD_ADD:
	case CMD_LOAD:
		extra_info.alu_result = SRC1VAL(EXECUTE) + SRC2VAL(EXECUTE);
		break;
	case CMD_STORE:
		extra_info.alu_result = SRC2VAL(EXECUTE) + extra_info.updated_dst_value;
		break;
	case CMD_SUB:
	case CMD_BREQ:
	case CMD_BRNEQ:
		extra_info.alu_result = SRC1VAL(EXECUTE) - SRC2VAL(EXECUTE);
		break;
	default:
		break;
	}
	//In case we need to calculate offset - for jumping
	if(OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ ||
			OPCODE(EXECUTE) == CMD_BRNEQ ) {
		extra_info.address_for_jump = mips_state.pc+extra_info.updated_dst_value;
	}
}

/**
 * An auxiliary function used for checking for load hazard.
 * Notice that load hazard is a very specific case, that only happens when we
 * have a command using the destination register of a load command exactly after
 * the load command. Anything else will be classified as data hazard, and not
 * load hazard.
 */
bool detect_load_hazard() {
	if(OPCODE(MEMORY) == CMD_LOAD) {
		//Case 1a: Src1 of EXECUTE is the same as DST of MEMORY
		if(DST(MEMORY) == SRC1(EXECUTE) && SRC1(EXECUTE) != 0 &&
		  (OPCODE(EXECUTE) != CMD_BR && OPCODE(EXECUTE) != CMD_HALT)) {
			return true;
		}
		//Case 1b: Src2 of EXECUTE is the same as DST of MEMORY
		if(DST(MEMORY) == SRC2(EXECUTE) && ISSRC2IMM(EXECUTE) == false && SRC2(EXECUTE) != 0 &&
		  (OPCODE(EXECUTE) != CMD_BR && OPCODE(EXECUTE) != CMD_HALT)) {
			return true;
		}
		//Case 2: DST of EXECUTE is the same as DST of MEMORY and this is a
		//branch command
		if(DST(MEMORY) == DST(EXECUTE) && DST(EXECUTE) != 0 &&
		  (OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ || OPCODE(EXECUTE) == CMD_BRNEQ)) {
			return true;
		}
	}
	return false;
}

/**
 * An auxiliary function used for after a branch resolution was positive.
 * It does the necessary actions:
 * 1. Clearing the pipe(Since we assumed branch not taken).
 * 2. Loading the correct PC into FETCH stage.
 * 3. Signaling we handled the jump.
 * After this function we expect a "return" to end the jump handling.
 */
void branch_taken() {
	insert_nop_to_stage(MEMORY);
	insert_nop_to_stage(EXECUTE);
	insert_nop_to_stage(DECODE);
	mips_state.pc = extra_info.address_for_jump;
	SIM_MemInstRead(mips_state.pc,&mips_state.pipeStageState[FETCH].cmd);
	extra_info.branch_resolution = false;
}

/**
 * An auxiliary function meant to help us update decode stage. This is a function
 * needed for the simulation of the MIPS computer, and it simulates the case
 * where information written to the register file is read.
 * This function will appear before any "return" prior to DECODE stage.
 */
void update_decode() {
	if(ISSRC2IMM(DECODE) != true) {
		SRC2VAL(DECODE) = GPR(SRC2(DECODE));
	} else {
		SRC2VAL(DECODE) = SRC2(DECODE);
	}
	SRC1VAL(DECODE) = GPR(SRC1(DECODE));
}

