#include "buttons.h"

enum Buttons linkButton(enum Buttons mqbButton)
{
	switch (mqbButton) {
		case MQB_NEXT_WINDOW: 	return PQ_NEXT_WINDOW;
		case MQB_PREV_WINDOW: 	return PQ_PREV_WINDOW;
		case MQB_MENU_UP: 		return PQ_MENU_UP;
		case MQB_MENU_DOWN: 	return PQ_MENU_DOWN;
		case MQB_RIGHT_WHEEL: 	return PQ_OK;
		case MQB_VOLUME_UP: 	return PQ_VOLUME_UP;
		case MQB_VOLUME_DOWN: 	return PQ_VOLUME_DOWN;
		case MQB_LEFT_WHEEL: 	return PQ_PHONE;
		case MQB_NEXT_TRACK: 	return PQ_NEXT_TRACK;
		case MQB_PREV_TRACK: 	return PQ_PREV_TRACK;
		case MQB_VOICE_CONTROL: return PQ_VOICE_CONTROL;
		case MQB_BACK:          return PQ_BACK;

		case MQB_ASSIST_BUTTON: return MQB_ASSIST_BUTTON;
		case MQB_STEERING_HEAT: return MQB_STEERING_HEAT;
		case MQB_CRUISE_SET: 	return MQB_CRUISE_SET;

		default: return NONE;
	}
}

enum Buttons decodePQButton(uint8_t buttonId)
{
	switch (buttonId) {
		case 2: 	return MQB_NEXT_TRACK;
		case 3: 	return MQB_PREV_TRACK;
		case 6: 	return MQB_VOLUME_UP;
		case 7: 	return MQB_VOLUME_DOWN;
		case 9: 	return MQB_PREV_WINDOW;
		case 10: 	return MQB_NEXT_WINDOW;
		case 26: 	return MQB_LEFT_WHEEL;
		case 34: 	return MQB_MENU_UP;
		case 35: 	return MQB_MENU_DOWN;
		case 40: 	return MQB_RIGHT_WHEEL;
		case 41: 	return MQB_BACK;
		case 42: 	return MQB_VOICE_CONTROL;
		default: return NONE;
	}
}

// Confirmed frame layout (matches real MQB gateway captures):
// [0] counter, [1] button 1 id, [2] button 2 id, [3] press duration 1,
// [4] wheel type (0xA3 Skoda / 0x90 VW), [5] press duration 2,
// [6] paddles, [7] misc/errors, [8] checksum
enum Buttons decodeMQBButton(uint8_t buttonId) {
	switch(buttonId) {
		case 0x02: return MQB_NEXT_WINDOW;   // right (src+)
		case 0x03: return MQB_PREV_WINDOW;   // left (src-)
		case 0x04: return MQB_MENU_UP;
		case 0x05: return MQB_MENU_DOWN;
		case 0x07: return MQB_RIGHT_WHEEL;   // OK
		case 0x10: return MQB_VOLUME_UP;
		case 0x11: return MQB_VOLUME_DOWN;
		case 0x15: return MQB_NEXT_TRACK;
		case 0x16: return MQB_PREV_TRACK;
		case 0x19: return MQB_VOICE_CONTROL;
		case 0x23: return MQB_BACK;          // view
		case 0x74: return MQB_CRUISE_SET;    // cruise mode button (center of left pod)
		default: return NONE;
	}
}

uint8_t getPQButtonCode(enum Buttons button)
{
	switch (button) {
		case PQ_VOICE_CONTROL: 	return 42;
		case PQ_PREV_TRACK: 		return 3;
		case PQ_NEXT_TRACK: 		return 2;
		case PQ_VOLUME_UP: 		return 6;
		case PQ_VOLUME_DOWN: 		return 7;
		case PQ_PHONE: 			return 26;
		case PQ_PREV_WINDOW: 		return 9;
		case PQ_NEXT_WINDOW: 		return 10;
		case PQ_MENU_UP: 			return 34;
		case PQ_MENU_DOWN: 		return 35;
		case PQ_OK: 				return 40;
		case PQ_BACK: 				return 41;
		default: return 0;
	}
}

void mqbToPq(uint8_t* input, uint8_t* output) {

	output[0] = (input[0] & 0x0F) | 0x80;
	output[1] = getPQButtonCode(linkButton(decodeMQBButton(input[1])));
	output[2] = 0x00;
	output[3] = 0x00;
	output[4] = 0x60;
	output[5] = 0x00;
	output[6] = 0x00;
	output[7] = 0x00;

	uint8_t id = 0x8E;
	output[8] = LIN_CalcCheckSum(&id, output, 8);
}