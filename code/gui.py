import flet as ft
from code.connection import servoConect
import time
import threading
import asyncio

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

    temp_text = []
    pos_text = []

    servo_feadback_value_lines = []
    for idx in range(6):
        name = ft.Text(value=f"Servo {idx+1}:", color = ft.Colors.BLACK)
        temp = ft.Text(value=f"Temeperatura serva {idx+1}", color = ft.Colors.BLACK)
        pos = ft.Text(value=f"Pozycja serva {conn.pos_array[idx]}", color = ft.Colors.BLACK)

        pos_text.append(pos)
        temp_text.append(temp)

        servo_feadback = ft.Row(
            controls=[name,temp,pos],
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
        width = 70
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

    main_widget = ft.Column(
        controls=[
            servo_position_input_bar,
            servo_feadback_baner,
            close_app_btn,
            speed_change_text
            ],
        alignment = ft.MainAxisAlignment.CENTER
        )

    page.add(main_widget)


    threading.Thread(target=conn.recvFeedback, daemon=True).start()

    async def update_gui():

        while True:
            for idx in range(6):
                temp_text[idx].value = f"Temp: {conn.temp_array[idx]}"
                pos_text[idx].value = f"Pos: {conn.pos_array[idx]}"

                """
                ------------------------------
                Part of disabel slider feature
                ------------------------------
                
                current_pos = int(conn.pos_array[idx])
                if current_pos != 0:
                    # Przeliczamy kroki (0-4095) z powrotem na stopnie (0-360) dla suwaka
                    slider_list[idx].value = (current_pos * 360) / 4095
                """
                    
            page.update()
            await asyncio.sleep(0.1)

    page.run_task(update_gui)


ft.run(main)