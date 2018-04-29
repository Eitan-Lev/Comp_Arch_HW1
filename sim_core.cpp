/* 046267 Computer Architecture - Spring 2018 - HW #1
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

//Defines&Macros
#define OPCODE(stage) mipsState.pipeStageState[stage].cmd.opcode
#define ISSRC2IMM(stage) mipsState.pipeStageState[stage].cmd.isSrc2Imm
#define GPR(number) mipsState.regFile[number]
#define SRC1(stage) mipsState.pipeStageState[stage].cmd.src1
#define SRC2(stage) mipsState.pipeStageState[stage].cmd.src2
#define DST(stage) mipsState.pipeStageState[stage].cmd.dst
#define SRC1VAL(stage) mipsState.pipeStageState[stage].src1Val
#define SRC2VAL(stage) mipsState.pipeStageState[stage].src2Val
#define WORDSIZE 4


//aux functions
void passCommand(pipeStage srcStage, pipeStage dstStage);
void nopToStage(pipeStage stage);
bool isDataHazard();
bool isLoadHazard();
void runAlu();
void takeBranch();
void updateDecode();

//A class to hold more information needed for MIPS opeartion
//the info class stores additional info neede for the MIPS to operate
class info {
public:
	//MEM/WB latches
	int wbFF;	//the alu's result will be stored here after one cycle
	int toWB; //info that will be written is stored here
	bool branchRes;	//stores the branchws resolution to use in wb
	//EXE/MEM latches
	int aluRes; //res of alu is stored here
	int jumpAddress; //the address we need to jump to is stored Here
	//WB
	int prevWBReg; //the reg last used in WB
	int toWrite;
	//variables
	bool isMemBusy;		 // if we  read from memory at the last cycle = true
	//For forwarding
	int memToExeWire;
	int wbToExeWire;
	int fwdBranch;
};
//static vars
static SIM_coreState mipsState;
static info moreInfo;

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instruction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {

	//PC is set to 0
	mipsState.pc = 0;

	//Reset all regs to 0
	for(int i = 0; i < SIM_REGFILE_SIZE; i++) {
		GPR(i) = 0;
	}

	//reset pipe stages
	for(int i = 1; i < SIM_PIPELINE_DEPTH; i++) {
		OPCODE(i) = CMD_NOP;
	}

	//Load the instruction in address 0x0 to pipe
	SIM_MemInstRead(0, &mipsState.pipeStageState[FETCH].cmd);

	return 0;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {

	/*************************************************************************/
	/* WB - Write back stage.
			write info to reg file. if we don't use split regfile */
	/*		then we write at the end of the cycle						 */
	/*************************************************************************/

	//we are not allowing ID stage to read the
	//information it needs in the previous cycle, because it wasn't written yet
	//if split reg file wasn't activated.
	moreInfo.prevWBReg = 0;
	if(split_regfile == false) {
		if((OPCODE(WRITEBACK) == CMD_ADD || OPCODE(WRITEBACK) == CMD_LOAD || OPCODE(WRITEBACK) == CMD_SUB
				|| OPCODE(WRITEBACK) == CMD_ADDI || OPCODE(WRITEBACK) == CMD_SUBI)
				&& DST(WRITEBACK) != 0) {
			GPR(DST(WRITEBACK)) = moreInfo.toWrite;
			moreInfo.wbToExeWire = moreInfo.toWrite;
			moreInfo.prevWBReg = DST(WRITEBACK);
		}
	}

	//when we jump on branch load new PC and use bubbles to clean the old one
	if(moreInfo.branchRes == true) {
		passCommand(MEMORY,WRITEBACK);
		takeBranch();
		return;
	}

	//clean the pipe if memory didn't finish reading.
	if(moreInfo.isMemBusy == true) {
		nopToStage(WRITEBACK);
	} else {
		if(split_regfile == true) {
			passCommand(MEMORY,WRITEBACK);
			switch(OPCODE(WRITEBACK)) {
			case CMD_ADD:
			case CMD_SUB:
			case CMD_ADDI:
			case CMD_SUBI:
			case CMD_LOAD:
				//Update the 'forwarding' unit
				//Write to reg file
				if(DST(WRITEBACK) != 0 ){
					GPR(DST(WRITEBACK)) = moreInfo.toWB;
					moreInfo.wbToExeWire = moreInfo.toWB;
				}
				break;
			default:
				break;
			}
		} else {
			//we just move the commands down the pipe if split reg file
			// wasn't activated
			passCommand(MEMORY,WRITEBACK);
			moreInfo.toWrite = moreInfo.toWB;
		}
	}

	/*************************************************************************/
	/* MEM - Memory stage. 		not all command continue past this stage						 */
	/*************************************************************************/

	if(moreInfo.isMemBusy == true) {
		// did memory finish?
		if(SIM_MemDataRead(moreInfo.wbFF, &moreInfo.toWB) == -1 ) {
			updateDecode();
			return;
		} else {
			moreInfo.isMemBusy = false;
			updateDecode();
			return;
		}
	} else {
		//if Mem isn't busy, handle command.
		passCommand(EXECUTE,MEMORY);
		switch (OPCODE(MEMORY)) {
			case CMD_ADD:
			case CMD_SUB:
			case CMD_ADDI:
			case CMD_SUBI:
				moreInfo.toWB = moreInfo.aluRes;
				moreInfo.memToExeWire = moreInfo.aluRes;
				break;
			case CMD_STORE:
				SIM_MemDataWrite(moreInfo.aluRes, SRC1VAL(MEMORY));
				moreInfo.memToExeWire = moreInfo.aluRes;
				break;
			case CMD_LOAD:
				//try to read, save the address if it takes more then one cycle.
				 if(SIM_MemDataRead(moreInfo.aluRes,&moreInfo.toWB) == -1 ) {
					 moreInfo.isMemBusy = true;
					 moreInfo.wbFF = moreInfo.aluRes;
				 }
				 break;
			case CMD_BR:
				moreInfo.branchRes = true;
				break;
			case CMD_BREQ:
				if(moreInfo.aluRes == 0) {
					moreInfo.branchRes = true;
				}
				break;
			case CMD_BRNEQ:
				if(moreInfo.aluRes != 0) {
					moreInfo.branchRes = true;
				}
				break;
			default:
				break;
		}
	}

	/*************************************************************************/
	/* EXE - Execute stage. also here we check for hazards and act accordingly
	/*************************************************************************/
 //allways pass the command. can't hurt.
	passCommand(DECODE, EXECUTE);
	moreInfo.fwdBranch = GPR(DST(EXECUTE));

	//check for load hazard
	if(isLoadHazard() == true) {
		nopToStage(EXECUTE);
		updateDecode();
		return;
	}
	//check fo rdata hazard
	if(isDataHazard() == true &&  forwarding == false) {
		nopToStage(EXECUTE);
		updateDecode();
		return;
	}
	//now everything is allright and we can run alu
	runAlu();

	/*************************************************************************/
	/* ID - DECODE stage.  */
	/*************************************************************************/
	passCommand(FETCH,DECODE);
	updateDecode();

	/*************************************************************************/
	/* IF - FETCH stage. increment PC and read instruction.			 */
	/*************************************************************************/
	nopToStage(FETCH);
	mipsState.pc += WORDSIZE;
	SIM_MemInstRead(mipsState.pc, &mipsState.pipeStageState[FETCH].cmd);
}


/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
	//Update PC
	curState->pc = mipsState.pc;
	//Update reg file
	for(int i = 0; i < SIM_REGFILE_SIZE ; i++) {
		curState->regFile[i] = GPR(i);
	}
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
 * this function implements the nop insertion method in case its needed
 */
void nopToStage(pipeStage stage) {
	ISSRC2IMM(stage) = false;
	DST(stage) = 0;
	SRC1VAL(stage) = 0;
	SRC2VAL(stage) = 0;
	OPCODE(stage) = CMD_NOP;
	SRC1(stage) = 0;
	SRC2(stage) = 0;
}

/**
 * this function check dor load hazards
 */
bool isLoadHazard() {
	if(OPCODE(MEMORY) == CMD_LOAD) {
		if(DST(MEMORY) == SRC2(EXECUTE) && ISSRC2IMM(EXECUTE) == false
			&& SRC2(EXECUTE) != 0 && (OPCODE(EXECUTE) != CMD_BR
			&& OPCODE(EXECUTE) != CMD_HALT)) {
			return true;
		}
		if(DST(MEMORY) == SRC1(EXECUTE) && SRC1(EXECUTE) != 0 &&
		  (OPCODE(EXECUTE) != CMD_BR && OPCODE(EXECUTE) != CMD_HALT)) {
			return true;
		}

		//branch
		if(DST(MEMORY) == DST(EXECUTE) && DST(EXECUTE) != 0 &&
		  (OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ || OPCODE(EXECUTE) == CMD_BRNEQ)) {
			return true;
		}
	}
	return false;
}
/**
 * this function passes command to the different pipe stages
 */
void passCommand(pipeStage srcStage, pipeStage dstStage) {
	mipsState.pipeStageState[dstStage].cmd =
							mipsState.pipeStageState[srcStage].cmd;
	SRC1VAL(dstStage) = SRC1VAL(srcStage);
	SRC2VAL(dstStage) = SRC2VAL(srcStage);
}

/**
 * this function checks for data hazards and indicates which data hazard is it
 */
bool isDataHazard() {
	bool flag = false;
	if((SRC2(EXECUTE) == DST(WRITEBACK) || SRC2(EXECUTE) == moreInfo.prevWBReg)
			&& SRC2(EXECUTE) != 0
			&& ISSRC2IMM(EXECUTE) == false
			&& OPCODE(WRITEBACK) != CMD_STORE ) {
		if(forwarding == true) {
			SRC2VAL(EXECUTE) = moreInfo.wbToExeWire;
		}
		flag = true;
	}

	if((DST(EXECUTE) == DST(WRITEBACK) || DST(EXECUTE) == moreInfo.prevWBReg)
			&& DST(EXECUTE) != 0 &&
			(OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ || OPCODE(EXECUTE) == CMD_BRNEQ || OPCODE(EXECUTE) == CMD_STORE)) {
		if(forwarding == true) {
			moreInfo.fwdBranch = moreInfo.wbToExeWire;
		}
	    flag = true;
	}
	if((SRC1(EXECUTE) == DST(WRITEBACK) || SRC1(EXECUTE) == moreInfo.prevWBReg)
			&& SRC1(EXECUTE) != 0
			&& OPCODE(WRITEBACK) != CMD_STORE ) {
		if(forwarding == true) {
			SRC1VAL(EXECUTE) = moreInfo.wbToExeWire;
		}
		flag = true;
	}

	if(SRC2(EXECUTE) == DST(MEMORY) && SRC2(EXECUTE) != 0 && OPCODE(EXECUTE) != CMD_BR
			&& OPCODE(EXECUTE) != CMD_NOP && ISSRC2IMM(EXECUTE) == false
			&& OPCODE(MEMORY) != CMD_BR && OPCODE(MEMORY) != CMD_BREQ && OPCODE(MEMORY) != CMD_BRNEQ
			&& OPCODE(MEMORY) != CMD_STORE ) {
		if(forwarding == true) {
			SRC2VAL(EXECUTE) = moreInfo.memToExeWire;
		}
		flag = true;
	}
	if(SRC1(EXECUTE) == DST(MEMORY) && SRC1(EXECUTE) != 0 && OPCODE(EXECUTE) != CMD_BR
			&& OPCODE(EXECUTE) != CMD_NOP
			&& OPCODE(MEMORY) != CMD_BR
			&& OPCODE(MEMORY) != CMD_BREQ
			&& OPCODE(MEMORY) != CMD_BRNEQ
			&& OPCODE(MEMORY) != CMD_STORE ) {
		if(forwarding == true) {
			SRC1VAL(EXECUTE) = moreInfo.memToExeWire;
		}
		flag = true;
	}

	if(DST(EXECUTE) == DST(MEMORY) && DST(EXECUTE) != 0 &&
			(OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ || OPCODE(EXECUTE) == CMD_BRNEQ || OPCODE(EXECUTE) == CMD_STORE)) {
		if(forwarding == true) {
			moreInfo.fwdBranch = moreInfo.memToExeWire;
		}
		 flag = true;
	}

	return flag;
}
/**
 * this function updates the decode stage
 */
void updateDecode() {
	if(ISSRC2IMM(DECODE) != true) {
		SRC2VAL(DECODE) = GPR(SRC2(DECODE));
	} else {
		SRC2VAL(DECODE) = SRC2(DECODE);
	}
	SRC1VAL(DECODE) = GPR(SRC1(DECODE));
}
/**
 * this function implements taking a branch.
 */
void takeBranch() {
	nopToStage(MEMORY);
	nopToStage(EXECUTE);
	nopToStage(DECODE);
	mipsState.pc = moreInfo.jumpAddress;
	SIM_MemInstRead(mipsState.pc, &mipsState.pipeStageState[FETCH].cmd);
	moreInfo.branchRes = false;
}
/**
 * this function implements the alu.
 */
void runAlu() {
	switch(OPCODE(EXECUTE)) {
	case CMD_ADD:
	case CMD_ADDI:
	case CMD_LOAD:
		moreInfo.aluRes = (SRC1VAL(EXECUTE) + SRC2VAL(EXECUTE));
		break;
	case CMD_STORE:
		moreInfo.aluRes = (SRC2VAL(EXECUTE) + moreInfo.fwdBranch);
		break;
	case CMD_SUB:
	case CMD_SUBI:
	case CMD_BREQ:
	case CMD_BRNEQ:
		moreInfo.aluRes = (SRC1VAL(EXECUTE) - SRC2VAL(EXECUTE));
		break;
	default:
		break;
	}
	//calc offset.
	if(OPCODE(EXECUTE) == CMD_BR || OPCODE(EXECUTE) == CMD_BREQ ||
			OPCODE(EXECUTE) == CMD_BRNEQ ) {
		moreInfo.jumpAddress = (mipsState.pc + moreInfo.fwdBranch);
	}
}
