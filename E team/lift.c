//#region config
#define L_USING_ENC    false
#define CH_USING_ENC   true
//#endregion

//#region positions
enum chainState  { CH_FIELD, CH_SAFE, STACK, CH_MIN, VERT, CH_MAX, CH_DEF };  //when chain bar is at CH_SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { 146,      146,     42,    0,      42,   197 };

enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
int liftPos[] = { 1425,  1430,    1880,   1420,       1880,   1880,   2960 };
//#endregion

//#region buttons
	//#subregion lift
#define f_liftUpBtn				Btn8U	//fielding mode
#define f_liftDownBtn			Btn8R
#define d_liftUpBtn       Btn5U	//driver load mode
#define d_liftDownBtn     Btn5D
	//#endsubregion
	//#subregion chain bar
#define f_chainInBtn			Btn5U	//fielding mode
#define f_chainOutBtn			Btn5D
#define d_chainInBtn			Btn8R	//driver load mode
#define d_chainOutBtn			Btn8U
	//#endsubregion
//#endregion
