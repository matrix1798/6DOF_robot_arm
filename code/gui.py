import flet as ft
from code.connection import servoConect
import time
import threading
import asyncio
from .tools.kinematicTools import robot6DOF

L_0, L_1, L_2, L_3, L_4, L_5 = 120.925, 54, 93.255, 27, 103.225, 0.0
robot = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

async def main(page: ft.Page):
    
    page.title = "6DOF robot menager"
    page.window.width = 800
    page.window.height = 600
    # in new Flet a method window.center() is in asynchronus loop
    await page.window.center()
    page.bgcolor = ft.Colors.WHITE

    try:
        conn = servoConect()
    except Exception as e:
        print(f"Port inizialization error: {e}")
        return

    def sendMessage(e):
        positions = []
        for idx in range(6):
            val = servo_pos_set_baner[idx].value

            if not val:
                current_pos = conn.pos_array[idx]
                
                if current_pos == 0 or current_pos == '0':
                    positions.append(2048) 
                else:
                    positions.append(current_pos)
                     
            else:
                # map value 0-360 to 0-4095
                val = int((int(val) * 4095 )/ 360)
                positions.append(val)

        try:
            speed = int(speed_change_text.value)

            if speed < 0:
                speed = 0
            if speed > 4095:
                speed = 4095

        except (ValueError, TypeError):
            print("Nieprawidłowa prędkość")
            return

        
        conn.sendMessage(positions,speed)
        
    #---------------------------
    # Set manual servos position
    #---------------------------

    servo_pos_set_baner = []
    for i in range(6):
        servo_pos_set_baner.append(ft.TextField(
            label= str(i+1),
            width=70
        ))

    btn_send_positions = ft.Button(
        content='Send',
        on_click= sendMessage
    )
    servo_pos_set_baner.append(btn_send_positions)

    servo_position_input_bar = ft.Row(
        controls=servo_pos_set_baner,
        spacing=10,
        alignment=ft.MainAxisAlignment.CENTER
    )

    #--------------------------------------------
    # Show feedback data from servo bus controler
    #--------------------------------------------

    temp_text = []
    pos_text = []
    voltage_text = []
    current_text = []

    servo_feadback_value_lines = []
    for idx in range(6):
        name = ft.Text(value=f"Servo {idx+1}:", color = ft.Colors.BLACK)
        temp = ft.Text(value=f"Temeperatura serva {idx+1}", color = ft.Colors.BLACK)
        pos = ft.Text(value=f"Pozycja serva {conn.pos_array[idx]}", color = ft.Colors.BLACK)
        voltage = ft.Text(value=f"", color = ft.Colors.BLACK)
        current = ft.Text(value=f"", color = ft.Colors.BLACK)

        pos_text.append(pos)
        temp_text.append(temp)
        voltage_text.append(voltage)
        current_text.append(current)

        servo_feadback = ft.Row(
            controls=[name,temp,pos,voltage,current],
            spacing= 20
            )
        servo_feadback_value_lines.append(servo_feadback)

    servo_feadback_baner = ft.Column(
        controls=servo_feadback_value_lines,
        spacing=20
    )

    #------------------------------
    # Button to change servos speed
    #------------------------------

    speed_change_text = ft.TextField(
        label = 'Predkosc:',
        value = '200',
        width = 70
    )

    #-----------------------
    # Invers kinematic baner
    #-----------------------

    def sendIK(e):
            positions = []

            try: 
                X = float(text_X_pos.value)
                Y = float(text_Y_pos.value)
                Z = float(text_Z_pos.value)
                roll = float(text_roll_pos.value)
                pitch = float(text_pitch_pos.value)
                yaw = float(text_yaw_pos.value)

                speed = int(speed_change_text.value)
                if speed < 0: speed = 0
                if speed > 4095: speed = 4095

            except(ValueError,TypeError):
                print('Nie poprawny format danych w IK')

            angles = robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')
            angles_conv = robot.angleConvert(angles)
            
            for idx in range(6):
                val_deg = angles_conv[idx]
                        
                val_mapped = int((val_deg * 4095 )/ 360)
                
                positions.append(val_mapped)
            
            conn.sendMessage(positions,speed)

    def checkIK(e):

            X = float(text_X_pos.value)
            Y = float(text_Y_pos.value)
            Z = float(text_Z_pos.value)
            roll = float(text_roll_pos.value)
            pitch = float(text_pitch_pos.value)
            yaw = float(text_yaw_pos.value)

            angles = robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')
            angles_conv = robot.angleConvert(angles)

            for i , angle in enumerate(angles_conv):
                compute_servo_pos_list[i].value = f"Servo {i+1}: {angle:.2f}"


    text_X_pos = ft.TextField(label = 'X', width = 70)
    text_Y_pos = ft.TextField(label = 'Y', width = 70)
    text_Z_pos = ft.TextField(label = 'Z', width = 70)
    text_roll_pos = ft.TextField(label = 'roll', width = 70)
    text_pitch_pos = ft.TextField(label = 'pitch', width = 70)
    text_yaw_pos = ft.TextField(label = 'yaw', width = 70)
    btn_IK = ft.Button(content = 'Send', on_click = sendIK)
    btn_check_IK = ft.Button(content='Check', on_click= checkIK)

    compute_servo_pos_list = []
    for _ in range(6):
        compute_servo_pos_list.append(ft.Text(value='', color=ft.Colors.BLACK))

    IK_baner_column = ft.Row(
        controls = [
            ft.Column(
                controls = [text_X_pos, text_Y_pos,text_Z_pos],
                alignment= ft.MainAxisAlignment.CENTER,
                horizontal_alignment= ft.CrossAxisAlignment.CENTER,
                spacing=10
                ),
            ft.Column(
                controls = [text_roll_pos,text_pitch_pos,text_yaw_pos],
                alignment= ft.MainAxisAlignment.CENTER,
                horizontal_alignment= ft.CrossAxisAlignment.CENTER,
                spacing=10                
                ),
            btn_IK,
            btn_check_IK,
            ft.Column(
                controls=compute_servo_pos_list,
                alignment= ft.MainAxisAlignment.CENTER,
                horizontal_alignment= ft.CrossAxisAlignment.CENTER,
                spacing=10  
            )
        ],
        spacing = 20
    ) 

    cont_IK_baner = ft.Container(
        content=IK_baner_column,
        width = 600,
        height = 250,
        border = ft.Border.all(1,"#172033"),
        alignment = ft.Alignment.CENTER,
        )

    """
    ------------------------------------------------------------
    Slabo dziala aktulanie przesyalnie wartosci poprzez slidery.
    Działa to ale strasznie szarpie przy zmianie.
    ------------------------------------------------------------

    slider_list = []

    def changeSlider(e):
        positions = []

        for idx in range(6):
            value = slider_list[idx].value

            #convert from angel to bit displey 4095/360

            val = int((int(value)*4095)/360)

            val = str(val)

            positions.append(val)

        conn.sendMessage(positions)

        # DODANE: Zaktualizuj suwaki, żeby 'nie zostały w tyle' za polami tekstowymi!
        for idx in range(6):
            slider_list[idx].value = (int(positions[idx]) * 360) / 4095
            
        page.update()

    for i in range(1,7,1):
        slider = ft.Slider(
            min = 0,
            max = 360,
            value = 180,
            label = f"Id: {i} " + "{value}",
            width = 800,
            divisions = 360,
            on_change = changeSlider
        )

        slider_list.append(slider)

    sliders_bar = ft.Column(
        controls = slider_list
    )
    """
    #--------------------
    # turn off app button
    #--------------------

    async def closeApp(e):
       await page.window.destroy()

    close_app_btn = ft.Button(
        content = "Zamknij program.",
        icon = ft.Icons.CLOSE,
        icon_color = ft.Colors.RED,
        on_click = closeApp
    )

    #------------
    # Main widget
    #------------

    main_widget = ft.Column(
        controls=[
            servo_position_input_bar,
            ft.Row(controls = [servo_feadback_baner,cont_IK_baner]),
            close_app_btn,
            speed_change_text
            ],
        alignment = ft.MainAxisAlignment.CENTER
        )

    page.add(main_widget)


    #-----------------
    # Module depends for asynchronic receive feedback data 
    # from servo controler and print in gui
    #-----------------

    threading.Thread(target=conn.recvFeedback, daemon=True).start()

    async def update_gui():

        while True:
            for idx in range(6):

                pos_in_angle = int(conn.pos_array[idx])

                pos_in_angle = pos_in_angle/4095 * 360

                temp_text[idx].value = f"Temp: {conn.temp_array[idx]}"
                pos_text[idx].value = f"Pos: {pos_in_angle:.2f} ({conn.pos_array[idx]})"
                voltage_text[idx].value = f"Volt: {conn.voltage_array[idx]}"
                current_text[idx].value = f"Current: {conn.current_array[idx]}"
                    
            page.update()
            await asyncio.sleep(0.1)

    page.run_task(update_gui)


ft.run(main)