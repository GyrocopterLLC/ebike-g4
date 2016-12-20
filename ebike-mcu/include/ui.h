#ifndef UI_H_
#define UI_H_

typedef enum
{
	UI_Idle,
	UI_MessageStart,
	UI_MessageEndLF,
	UI_MessageEndCR
} UI_MessageState;

typedef enum
{
	UIMSG_Null = 0,
	UIMSG_USB
} UI_Messages;

typedef struct
{
	UI_MessageState State;
	UI_Messages Message;
} UI_Type;

void UI_Process(uint8_t nextbyte);







#endif // UI_H_
