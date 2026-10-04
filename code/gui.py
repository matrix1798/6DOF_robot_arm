import flet as ft

async def main(page: ft.Page):

    page.title = "6DOF robot menager"
    page.window.width = 800
    page.window.height = 600
    # in new Flet a method window.center() is in asynchronus loop
    await page.window.center()
    page.bgcolor = ft.Colors.WHITE

    def test_fun ():
        print(f'Wartos 3: {servo_pos_set_baner[2].value}')
        page.update()

    servo_pos_set_baner = []
    for i in range(6):
        servo_pos_set_baner.append(ft.TextField(
            label= str(i+1),
            width=70
        ))

    btn_send_positions = ft.Button(
        content='Send',
        on_click= test_fun
    )
    servo_pos_set_baner.append(btn_send_positions)

    servo_position_input_bar = ft.Row(
        controls=servo_pos_set_baner,
        spacing=10,
        alignment=ft.MainAxisAlignment.CENTER
    )

    servo_feadback_value_lines = []
    for idx in range(6):
        name = ft.Text(value=f"Servo {idx+1}:")
        temp = ft.Text(value=f"Temeperatura serva {idx+1}")
        pos = ft.Text(value=f"Pozycja serva {idx+1}")
        servo_feadback = ft.Row(
            controls=[name,temp,pos],
            spacing= 20,
            alignment= ft.MainAxisAlignment.CENTER
            )
        servo_feadback_value_lines.append(servo_feadback)

    servo_feadback_baner = ft.Column(
        controls=servo_feadback_value_lines,
        spacing=20
    )

    main_widget = ft.Column(
        controls=[
            servo_position_input_bar,
            servo_feadback_baner
            ],
        alignment = ft.MainAxisAlignment.CENTER
        )

    page.add(main_widget)


ft.run(main)