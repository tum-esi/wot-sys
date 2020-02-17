# Sequence Diagrams

Each sequence diagram has an own mashup. You can find these in the mashup directory. Below you can see the same description you can find commented in the mashup itself.

## Mashups using EVENTS for triggering ACTIONS

### mashup_dobotmagician_basic_full

Description:

This mashup contains the following things:
- two color sensors
- two infrared sensors
- two conveyor belts, moved by stepper motors
- one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. At the 
end of the moving operation, the conveyor belt restarts automatically again. 

![mashup_dobotmagician_basic_full](http://www.plantuml.com/plantuml/svg/xLTHRzem47xthx2GaBe7fCQFrPWwO9lwa1Mj-mDEV827yKMsGzlz-fsJPPXgC5AfKgVoG8XztttV-SZ7ATS-I1VgZM4ha0eSBwJN9L_Knh05Og2TkLm27rPq7N9SyA6HFZGhne8E1dWk_QgkqapcrzfMqiaDPRkWIrr96-PIAMsNN5mcyGa-TFlP9MFL3lwRTuesE3HSUfvFz-8pB33aSabhQMDybZDMu0CVH56U3VRGnBE5HtGJC26BwEaasmFSs8MJ3jGTM8-kWTpSPKyu3BeaFgKuMCL9D5KMXvL5gooEA8jMMHnI5gsos2cpyPWuTD3Iw9yoQBHyF6Oid_BH1shA5TTsYsluMCRGnQ3voUVeTLmEtZ59-riP8FPlbdSv2Yqm1KcmOwJ6_PKodkypkdlfLCfAy3-9eb-YOCmWLXoiCk1zKy3xqOYwGnNukl2bqmLyse8D5mC50Se0whRuGTz4TAHQhA8cOHIaTHviMQNBTSlRzItRRrnNsO7E7UBTu1Q-O_fOzQUDJ-5JNWTIDQmf7U4wFhfj2Vcz3Yj-MuUg3bTyyzZUDQbP54jpbmx9vasCAwZ0Ah21IHFuDr17ZzYsZb2zJWV60__Y0Em9v0skF_t4r702UuJr9_-AzXHoWlKajof-dc_7n4YYXv74gKOIFOmaNiP8ubb64YSQIUoC9Fvf9B5l9F4wZNGsKl_nzDyEf9V_0pYFlDVarFF8UnCJwJoHpXCfPhDhkiQN-by0)

### mashup_dobotmagician_detect&stop

Description:

This mashup contains the following things:
- two infrared sensors
- two conveyor belts, moved by stepper motors

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object.

![mashup_dobotmagician_detect&stop](http://www.plantuml.com/plantuml/svg/bPB1RjGm48RlynIZI8lAOQN6nugWiX58FQoer2UOn9FMrFLOzYGKdXux2sptm8gzn8hdc-yVsRxDIadd5mzFJ9OJ3fJTYDFiFKmYMdRkwP5n4yjtWpBXnbFMzGz0dNh6FUMdEQ9bvL7VPPK8a9zTY9JefRGdTQEB5FIUh7NX4Strc_fENl_ijzS0yOZ_5lSID8b7ohZl0GPvnQPAiMrEwC8j_4FIZhssbUvtxGbm5wP4YUq3XonfHUuUMk1WiKZVcis_pUPWDcVCvc0sHpDqNM6SElBk9wcJW5q7iEzn-x6EUOCkBFBCdyPQkshMIwbfNoHzfsIRzq2bj91onVysciiQ3O0NYSZ1UiuPgkV3TbjFuGRpFEGnkO4_BnpqgZbS9zklmxUo5j6PQINMpC9OFfdp660k233_6s1WFMA0sxBKD_mB)

### mashup_dobotmagician_detect&stop&color

Description:

This mashup contains the following things:
- two color sensors
- two infrared sensors
- two conveyor belts, moved by stepper motors
- one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Regardless the color, 
the robot moves the object still to the same specified position. After that the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. 
At the end of the moving operation, the conveyor belt restarts automatically again. 

![mashup_dobotmagician_detect&stop&color](http://www.plantuml.com/plantuml/svg/xLPHRzem47xthx2GgBe7fCQFQ4mTOPFwa1M9_G4dFi33-2pxGjlz-jd968asM46ggarz88bzttsVx_pfn6qWwQdU6hO6gS3pKWPTyMLj35iYKjoPonNmeOlF8SSb7nePg5anHfeCy4A6TUsu0eAAhWAXkwhGe6SiRBHrqijj9F6aA-safRbKIjiL5pUT-1GVz_lP3MFk0Fz9dwCbZuRBm8ky5vzXYLJ8LLnBc-ApWh4I7_aWYV9iq4D7dXqyeP-2eIoXysdM0zpPfPSUr09iGDz0xXPzI9wgwyJp60UhU1JjAelZog9L5YUKHQiiZYcBLbaSbDba4ZaqQMdqTqaQBPzC62joFleGcpBcskvm0n-h5BeUD3S_nw3JSl2EoRY_amIfVxDYdwFG0bFG3RQCgN5_f4prVKR_8Btgi2RuhqHnNg9Wp20w3bOP2A4vmFlHADtBc8Uw39NN9NpQWQNhGMj3KFVbj_YEH2TEYovg9eo2xZcFjipfQjFoxlkMzHktFzcHpXtZtU8ElcBNLXVGvelVAlGWLSFQeKkuFL_yR0dvWwOrlw_9rJJcswVsgqdDabWtT-MnTgqJevm21_5XYKb3_0jQE5bbsxq4Ewl2A95-bpklunbnXcV4SpqZpl2CU1dFY8iy8vxf6N7mZFYhPqJVC-8rFNFnqFbdnypBU_tL1jdR8FllvipRd4dPdB7R-4f_n7y0)

### mashup_dobotmagician_detect&stop&grip&drop

Description:

This mashup contains the following things:
- two infrared sensors
- two conveyor belts, moved by stepper motors
- one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves the object still to the same specified position. 
After that the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. At the end of the moving operation, the conveyor belt 
restarts automatically again. 

![mashup_dobotmagician_detect&stop&grip&drop](http://www.plantuml.com/plantuml/svg/xPPHYnen4CVVvrC4GRa-2BTvF6gvgYtSmxR2VOBiPlHIOoOaextsqtUYtTuAzUWM-Y3qmLsI-Szl9l-CiFUPJUBzpiCJ6ej9DIQxLgtttiEQY6LdPJQenb6UOqLhDVOcyt45m8uzgjhaftrK5XbRdcIcEDaa5oSsKGJ8MnUYIMOdeCIkTT44NXbhNTWeVTkBp-cvswzk0UAB_6U91GLEv9N9gbwSnPVK4DTc8siJIdnP0pJqh4QbgAf6PshX7F0RfJbwhefwCQ_E10zXdKn2-uWXKpfA7Xuhm62LGFjaVPciJsJz2bcVoFg53BEPQ1mxuzrtmuw2ciq0weMQlfD3tIaN3hJ5zsq9tOoEjxUYxCfoz0QCx1yCeqXXMNSvbWA2nLwm8yh1Vuii_N-azDKaswUA_5UY7fQe0JnHL1YinvoXSDvEfyNNEvNtJMwJQ_330GFVZ4wZXFPpyqNU0dgbMufIisWizlkyQ5bqxVR4xNohpethEhlWt2QH3Fk2F2KXMumeZy1WFAjF8hzOU4S7F1Kkia4rfKJv_VsbzUpM0-pMVsgt7c2tlZgxXu_wjGxtFn-JA_txz9lwFzsIhG3kvLK-77u0)

### mashup_dobotmagician_detect&stop&movebelt

Description:

This mashup contains the following things:
- two infrared sensors
- two conveyor belts, moved by stepper motors

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the corresponding conveyor belt restarts after some time.

![mashup_dobotmagician_detect&stop&movebelt](http://www.plantuml.com/plantuml/svg/dPF1Rjim38RlV0h2G8hkaA0MiCkm35s23UWXM84-0MqnhLP5D2J6w_RqevnjQGuBsbuicFpryRC5NHUbhFjTX0ScpnaxAg77xJv6s8geLMxfdd4ss7E6iiLPfABJ6u06ZOmRAW_x0JqhztfHL8QBdOpSSLI0yXZIG9bsniaQ-Z1GqbloFgHxT5VF-Yjv-bjlhm26O_pFXBKapHAH2cxM09qyOLEXs3OdQSkD_5FooWJQcbwlsfF0JTfcokplE1N9K-JchWLE7WtwdEp-JtO7iZj3TWUoEv9XkRHCq40n_28Dad2v1DYiSVwfQdx0a4PvvCzzxLqsq-dKQLyb_w3ictT0rXf9kSR_RNIls-W0eiY0vXMvb4dWutnkhcPGzbtfS-ZuoyX9Bvl3kRB_rdstrK1dR6MeCotZ-SHpiLYynyAz95ymMZLRvUsVVVHrh_1rB_LrPtpVzhSvUGIujgLUjTy0)

### mashup_dobotmagician_only1side

Description:

This mashup contains the following things:
- one color sensors
- one infrared sensors
- one conveyor belts, moved by stepper motors
- one dobot magician

The conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that the robot is ready for a new command from the conveyor belt. At the end of the moving operation, the conveyor belt 
restarts automatically again. 

![mashup_dobotmagician_only1side](http://www.plantuml.com/plantuml/svg/xPDFRziy3CRl-XH4W09z3mtgzrWiGvTa6thm6gpx0hB595faKP3ejDcd7-NCc5CiHOBjc8F_Y7puenu9l4-i8xUDKslK1YDKEjaQbgrpQad44bde5S9LaFSLq1Akd4xShPHYomwXr6dT1Y3lTaMo1fLA6-k3ZhgHuiYsja5xNcXZh5_1_xU3_9HU-dXngrJuBV-5dP7dI0vqWd9sa9zJHLpgbQorp_bvgLH5Bp3AJQ4O7QY5iyKTnIawBh9wDYqE10z-6NL4yuG-KUma3q_5AuQZECZF98_UW3HLQZ8HX6MhdVsXsPA7oKIfSWOtxsLdTs3zbZRueSwfwr5tx0jADYz7_oajyQrcp2RcPLzZoACoE4ZsHDdz7v7PtYUAppgQ8JMRLOue07hZCAMEy-xcHZmAABLLggEjyECMFL-F33BMZEQn-YvV0RtHb4BkAHg3mtP7dGTRR_RStdvnwB_VsP43ECPjQ8lVQ7X5vsFpZRxcHjIcemvnekjvyhi7mhFbDJos75g-WsQt_-jA2sNme7OLgGtGvS1WG6_GizAEuIkQDotkZqvKPxb3bn0-HqH_0hpJdO-VkXPFe6VP-V0lv4-1Y-nCk3Vv-Okf6e_7e9UC4H8r26pb9R5_CM2N0RiCs6N0f1gKkfTFspZr4m00)

## Mashups using SETINTERVAL for triggering the ACTIONS

### mashup_dobotmagician_polling

This is a mashup that uses the devices of the dobot magician setup and is based on SETINTERVAL for triggering the ACTIONS.

Description:

This mashup contains the following things:
- two color sensors
- two infrared sensors
- two conveyor belts, moved by stepper motors
- one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that, the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. At the 
end of the moving operation, the conveyor belt restarts automatically again. 

![mashup_dobotmagician_polling](http://www.plantuml.com/plantuml/svg/xLN1JXin4BtxAqQHaEW1AMkf5sWgcaMjE6o9IjNx99u45yTZsTv0-lMrTmbie84iRQKUUiWczdlpdZsJfpqD4NsibqPS4IhoCCMWPp2lZH5pvfXs9hWWE73fUG0yXmE38JOh8QAEXg32S5KxS6oCjWiXmhMs3ZqkKwcFUgOTsZX1fH8ASjZ1ntoxsI-6GhW7-freoJPwDe01gd8BF-CfnmeNQOqsusUL45E-XK4sXMAmnKuwArgp7vE9HMQNusABS6xd7ZsfIxA1VKCvlomUQHZs7RnCE5a5oRJhB7SxozPPFkCiMsUvorcsplB1MOn6IKD7ZKR_mAZPmcWaH5N2qVlKb6FGTiNNz66MeSD1C-y91vsNWpS2q_uA8-N-dLMR6iMMXA8Ek55CZVkbPBxVH_Otw5LNDT7l2sM_GYc4ONP0LXaAuMxboH56e1NvDRmTmZAq-dbSnmb5DV7ioCVruO2dtsaM9vu2sHabzSHyTtJKKhT1WIO-seBKsCtXg_8Obkjlw644SpI1Jfvh1xjygUIdgDk8dJDnUdPzqNXl1bCycanxy5sZsQMxv1LzvSw_vpMovUMJ6-OkDwfTkSJRw3KbMH1kTBo2YpgwEkO6jh-QqYARTMiNdci73GQA75b5DZPZ-KBgsIkshKkiNhSZ4mW-UIAxXtZ3woy_DZNjePvf_SK_izr7FD5wYbkLFx-Rzsvaeg_ff3trqumDxnrVZhZSF-BopqLSxXjnsIFYykz4NBugudBFYCk7YCiN8owt8ox_qOZ_ZtZ_D_Fh8vuU8CHf-ggNHlm4)

