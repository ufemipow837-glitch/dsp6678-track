-heap  0x800
-stack 0x8000

MEMORY
{
	/* Local L2, 0.5~1MB*/
	VECTORS: 	o = 0x00800000  l = 0x00000200
	LL2_RW_DATA: 	o = 0x00800200  l = 0x0004FE00

	/* Shared L2 2~4MB*/
	SL2: 		o = 0x0C000000  l = 0x00200000
    MSMCSRAM:   o = 0xC0000000, l = 0x00000800
	/* External DDR3, upto 2GB per core */
	DDR3_CODE: 	o = 0x80000000  l = 0x01000000   /*set memory protection attribitue as execution only*/
	DDR3_R_DATA: 	o = 0x81000000  l = 0x01000000 	 /*set memory protection attribitue as read only*/
	DDR3_RW_DATA: 	o = 0x82000000  l = 0x06000000   /*set memory protection attribitue as read/write*/
	EMIF16_DATA:    o = 0x74000000  l = 0x4000000  /* EMIF16 memory space */
}

SECTIONS
{
	vecs       	>    VECTORS 

	.text           >    DDR3_CODE
	.cinit          >    LL2_RW_DATA
	.const          >    LL2_RW_DATA
	.switch         >    LL2_RW_DATA

	.stack          >    LL2_RW_DATA
	GROUP
	{
		.neardata
		.rodata
		.bss
	} 		>    LL2_RW_DATA
	.data    >   LL2_RW_DATA
	.datag          >    EMIF16_DATA
	.rec            >    DDR3_CODE
	.far            >    LL2_RW_DATA
	.fardata        >    LL2_RW_DATA
	.cio            >    LL2_RW_DATA
	.sysmem         >    LL2_RW_DATA
	.mySection      >    DDR3_CODE
     .far:DDR        >    DDR3_RW_DATA
}


