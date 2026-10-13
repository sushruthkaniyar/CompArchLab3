/*
    Name 1: Sushruth Kaniyar
    UTEID 1: ssk3346
*/

/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                

  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%.4x\n", BUS);
    printf("MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/************************************************************/
/*                                                          */
/* Procedure : init_memory                                  */
/*                                                          */
/* Purpose   : Zero out the memory array                    */
/*                                                          */
/************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {                   
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *ucode_filename, char *files[], int num_prog_files) { 
    int i;
    init_control_store(ucode_filename);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	    load_program(files[i]);
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                              
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv[1], &argv[2], argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code.
   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/

int sext(int value, int bits) {
    int mask = (1 << bits) - 1;
    value &= mask;
    if (value & (1 << (bits - 1))) {
        value |= ~mask;     
    }
    return value;
}

void eval_micro_sequencer() {

  /* 
   * Evaluate the address of the next state according to the 
   * micro sequencer logic. Latch the next microinstruction.
   */

    if(GetIRD(CURRENT_LATCHES.MICROINSTRUCTION)){
        NEXT_LATCHES.STATE_NUMBER = 0x0F & (CURRENT_LATCHES.IR >> 12);
    } 
    else{
        int cond = GetCOND(CURRENT_LATCHES.MICROINSTRUCTION);
        int J = (GetJ(CURRENT_LATCHES.MICROINSTRUCTION));
        int branch = ((cond == 2) & CURRENT_LATCHES.BEN) << 2;
        int ready = ((cond == 1) & CURRENT_LATCHES.READY) << 1;
        int addrMode = ((cond == 3) & ((CURRENT_LATCHES.IR >> 11) & 1));

        NEXT_LATCHES.STATE_NUMBER = J | branch | ready | addrMode;
    }
    memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[NEXT_LATCHES.STATE_NUMBER], sizeof(int) * CONTROL_STORE_BITS);
}

int cycle_num = 1;
void cycle_memory() {
 
  /* 
   * This function emulates memory and the WE logic. 
   * Keep track of which cycle of MEMEN we are dealing with.  
   * If fourth, we need to latch Ready bit at the end of 
   * cycle to prepare microsequencer for the fifth cycle.  
   */

    if((cycle_num == 4) && (GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION))){
        NEXT_LATCHES.READY = 1;
        cycle_num++;
    }
    else if((cycle_num == 5) && (GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION))){
        cycle_num = 1;
        NEXT_LATCHES.READY = 0;
        
        if(GetR_W(CURRENT_LATCHES.MICROINSTRUCTION)){
            int word = CURRENT_LATCHES.MAR >> 1;
            int size = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
            int MAR_0 = CURRENT_LATCHES.MAR & 1;

            if(size){
                MEMORY[word][0] = CURRENT_LATCHES.MDR & 0xFF;
                MEMORY[word][1] = (CURRENT_LATCHES.MDR >> 8) & 0xFF;
            }else if(MAR_0){
                MEMORY[word][1] = (CURRENT_LATCHES.MDR >> 8) & 0xFF;
            }else{
                MEMORY[word][0] = CURRENT_LATCHES.MDR & 0xFF;
            }
        }
    }
    else if((GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION))){
        cycle_num++;
    }
    else{
        cycle_num = 1;
    }

}


int GATE_PC_VAL, GATE_MDR_VAL, GATE_ALU_VAL, GATE_SHF_VAL, GATE_MARMUX_VAL, addr_val;
void eval_bus_drivers() {

  /* 
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers 
   *             Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */   

    // gate_marmux
    int marmux = GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int val1;
    int val2;
    if(GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION)){
        int sr1mux = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
        int baseR = 0;
        if(sr1mux){
            baseR = (CURRENT_LATCHES.IR >> 6) & 0x7;
        }else if (!sr1mux){
            baseR = (CURRENT_LATCHES.IR >> 9) & 0x7;
        }
        val1 = CURRENT_LATCHES.REGS[baseR];
    }
    else{
        val1 = CURRENT_LATCHES.PC;
    }
    int addr2mux = GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    if(addr2mux == 0){
        val2 = 0;
    }else if(addr2mux == 1){
        val2 = sext((CURRENT_LATCHES.IR & 0x3F), 6);
    }else if(addr2mux == 2){
        val2 = sext((CURRENT_LATCHES.IR & 0x1FF), 9);
    }else if(addr2mux == 3){
        val2 = sext((CURRENT_LATCHES.IR & 0x7FF), 11);
    }
    int lshf1 = GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION);
    if(lshf1){
        val2 <<= 1;
    }
    addr_val = Low16bits(val1 + val2);

    if(!marmux){
        int val = (CURRENT_LATCHES.IR & 0xFF);
        GATE_MARMUX_VAL = (val << 1);
    }else if(marmux){
        GATE_MARMUX_VAL = addr_val;
    }


    // gate_pc
    GATE_PC_VAL = CURRENT_LATCHES.PC;


    // gate_alu
    int valA;
    int valB;
    int aluk = GetALUK(CURRENT_LATCHES.MICROINSTRUCTION);
    int sr1mux = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int baseR1 = 0;
    int baseR2 = 0;

    if(sr1mux){
        baseR1 = (CURRENT_LATCHES.IR >> 6) & 0x7;
    }else if (!sr1mux){
        baseR1 = (CURRENT_LATCHES.IR >> 9) & 0x7;
    }
    valA = sext(Low16bits(CURRENT_LATCHES.REGS[baseR1]), 16);

    int sr2mux = (CURRENT_LATCHES.IR >> 5) & 1;
    if(sr2mux){
        valB = sext(CURRENT_LATCHES.IR, 5);
    }else if(!sr2mux){
        baseR2 = (CURRENT_LATCHES.IR & 0x7);
        valB = sext(Low16bits(CURRENT_LATCHES.REGS[baseR2]), 16);
    }

    if(aluk == 0){
        GATE_ALU_VAL = Low16bits(valA + valB);
    }else if(aluk == 1){
        GATE_ALU_VAL = Low16bits(valA) & Low16bits(valB);
    }else if (aluk == 2){
        GATE_ALU_VAL = Low16bits(valA) ^ Low16bits(valB);
    }else if (aluk == 3){
        GATE_ALU_VAL = Low16bits(valA);
    }

    // gate_shf
    int bit4 = (CURRENT_LATCHES.IR >> 4) & 1;
    int bit5 = (CURRENT_LATCHES.IR >> 5) & 1;
    int amt  = CURRENT_LATCHES.IR & 0xF;

    int SR = (CURRENT_LATCHES.IR >> 6) & 0x7;

    if(!bit4){
        GATE_SHF_VAL = Low16bits(CURRENT_LATCHES.REGS[SR] << amt);
    }else if ((bit4) && (bit5)){ // arithmetic
        int val = CURRENT_LATCHES.REGS[SR] & 0xFFFF;
        if (val & 0x8000){
            val |= 0xFFFF0000;
        }
        GATE_SHF_VAL = Low16bits(val >> amt);
    }else if ((bit4) && (!bit5)){ // logical
        GATE_SHF_VAL = Low16bits((CURRENT_LATCHES.REGS[SR] & 0xFFFF) >> amt);
    }

    // gate_mdr
    int MAR_0 = (CURRENT_LATCHES.MAR & 1);
    int data_size = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);

    if(data_size){
        GATE_MDR_VAL = CURRENT_LATCHES.MDR;
    }else if (!MAR_0){
        int val = (CURRENT_LATCHES.MDR & 0xFF);
        GATE_MDR_VAL = Low16bits(sext(val, 8));
    }else{
        int val = ((CURRENT_LATCHES.MDR >> 8) & 0xFF);
        GATE_MDR_VAL = Low16bits(sext(val, 8));
    }

}


void drive_bus() {

  /* 
   * Datapath routine for driving the bus from one of the 5 possible 
   * tristate drivers. 
   */  

    int gatepc = GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION);
    int gatemdr = GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION);
    int gatealu = GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION);
    int gatemarmux = GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int gateshf = GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION);

    if(gatepc){
        BUS = GATE_PC_VAL;
    }else if(gatemdr){
        BUS = GATE_MDR_VAL;
    }else if(gatealu){
        BUS = GATE_ALU_VAL;
    }else if(gatemarmux){
        BUS = GATE_MARMUX_VAL;
    }else if(gateshf){
        BUS = GATE_SHF_VAL;
    }else{
        BUS = 0;
    }

}


void latch_datapath_values() {

  /* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */       

    // ld.mar
    int ldmar =  GetLD_MAR(CURRENT_LATCHES.MICROINSTRUCTION);
    if(ldmar){
        NEXT_LATCHES.MAR = BUS;
    }

    // ld.mdr
    int ldmdr =  GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION);
    if(ldmdr){
        int mioen = GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION);
        if(mioen){
            if((CURRENT_LATCHES.READY)){
                NEXT_LATCHES.MDR = Low16bits((MEMORY[(CURRENT_LATCHES.MAR >> 1)][0] & 0XFF) | ((MEMORY[(CURRENT_LATCHES.MAR >> 1)][1] & 0XFF) << 8));
            }
        }else{
            int datasize = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
            if(datasize){
                NEXT_LATCHES.MDR = BUS;
            }else{
                NEXT_LATCHES.MDR = (BUS & 0xFF) | ((BUS & 0xFF) << 8);
            }
        }
    }

    // ld.pc
    int ldpc =  GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION);
    if(ldpc){
        int pcmux = GetPCMUX(CURRENT_LATCHES.MICROINSTRUCTION);
        if(pcmux == 0){
            NEXT_LATCHES.PC = Low16bits(CURRENT_LATCHES.PC + 2);
        }else if(pcmux == 1){
            NEXT_LATCHES.PC = BUS;
        }else if(pcmux == 2){
            NEXT_LATCHES.PC = addr_val;
        }
    }

    // ld.ir
    int ldir =  GetLD_IR(CURRENT_LATCHES.MICROINSTRUCTION);
    if(ldir){
        NEXT_LATCHES.IR = BUS;
    }

    // ld.cc
    int ldcc =  GetLD_CC(CURRENT_LATCHES.MICROINSTRUCTION);
    if(ldcc){
        int bus16 = sext(BUS, 16);
        if(bus16 == 0){
            NEXT_LATCHES.N = 0;
            NEXT_LATCHES.Z = 1;
            NEXT_LATCHES.P = 0;
        }else if(bus16 > 0){
            NEXT_LATCHES.N = 0;
            NEXT_LATCHES.Z = 0;
            NEXT_LATCHES.P = 1;
        }else if(bus16 < 0){
            NEXT_LATCHES.N = 1;
            NEXT_LATCHES.Z = 0;
            NEXT_LATCHES.P = 0;
        }
    }

    // ld.reg
    int ldreg =  GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION);
    if(ldreg){
        if(GetDRMUX(CURRENT_LATCHES.MICROINSTRUCTION)){
            NEXT_LATCHES.REGS[7] = BUS;
        }else{
            int reg = ((CURRENT_LATCHES.IR >> 9) & 0x7);
            NEXT_LATCHES.REGS[reg] = BUS;
        }
    }

    // ld.ben
    int ldben = GetLD_BEN(CURRENT_LATCHES.MICROINSTRUCTION);
    if(ldben){
        int n = ((CURRENT_LATCHES.IR >> 11) & 1) & CURRENT_LATCHES.N;
        int z = ((CURRENT_LATCHES.IR >> 10) & 1) & CURRENT_LATCHES.Z;
        int p = ((CURRENT_LATCHES.IR >> 9) & 1) & CURRENT_LATCHES.P;

        NEXT_LATCHES.BEN = (n | z | p);
    }

}