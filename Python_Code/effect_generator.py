from helper import *
from ip_discovery import *
import time
from pythonosc.osc_server import AsyncIOOSCUDPServer
from pythonosc.dispatcher import Dispatcher
import asyncio
import sys
import supriya
from supriya.ugens import EnvGen, Out, SinOsc
from supriya.synthdefs import Envelope, synthdef


# add custom ip config creator in code

sc_server = supriya.Server().boot()

@synthdef()
def simple_sine(frequency=440, amplitude=0.1, gate=1):
    sine = SinOsc.ar(frequency=frequency) * amplitude
    envelope = EnvGen.kr(envelope=Envelope.adsr(), gate=gate, done_action=2)
    Out.ar(bus=0, source=[sine * envelope] * 2)
# supriya.graph(simple_sine)

_ = sc_server.add_synthdefs(simple_sine)
group = sc_server.add_group()
for i in range(3):
    print(i)
    _ = group.add_synth(simple_sine, frequency=111 * (i + 1))

def print_handler(address, *args):
    print(f"{address}: {args}")


def default_handler(address, *args):
    print(f"DEFAULT {address}: {args}")


dispatcher = Dispatcher()
dispatcher.map("/TorqueTuner_001/instrument/velocity", print_handler)
# dispatcher.set_default_handler(default_handler)


ip = ip_selection(interfaces())
if ip is None:
    print("Not Connected to a network, quiting program.")
    exit()
port = 8000

plt.ion()
button_x = 0
button_y = 0

init_H = np.sqrt(27)/2
init_k = 27/(init_H**3*4)
pressure_difference = np.linspace(-0.013,1.2,100)
mouth_and_incoming_pressure_difference = np.linspace(0,1.2,100)
initial_exponent_1 = 1/2
initial_exponent_2 = 3/2
pressure_table = [pressure_difference,mouth_and_incoming_pressure_difference]
functable = [[partial(reedFlow_Equation,pressure_difference,initial_exponent_1,initial_exponent_2), partial(linearized_reedFlow_Equation,pressure_difference)],[partial(pressure_difference_solution,mouth_and_incoming_pressure_difference,pressure_difference)]]

def array_functions_generator(functable,H,k,x,y):
    # print("Line Recalculated")
    return functable[x][y](H,k)

gs_kw = dict(width_ratios=[2,2], height_ratios=[1])
figs, axs = plt.subplots(ncols=2,nrows=1, figsize=[12,5],gridspec_kw=gs_kw, sharey=True)

axs[0].set_title('Felt effect')
axs[1].set_title('Output effect')
axs[0].set_xlabel("Normalized Angle")
axs[1].set_xlabel("Normalized Angle")
axs[0].set_ylabel("Normalized Output")

line1, = axs[0].plot(pressure_table[button_x],array_functions_generator(functable,init_H,init_k,button_x,button_y))
line2, = axs[1].plot(pressure_difference,array_functions_generator(functable,init_H,init_k,0,0))


figs.subplots_adjust(bottom=0.4)

rect_ax1 = [0.115, 0.25, 0.35, 0.03]
rect_ax2 = [0.535, 0.25, 0.35, 0.03]

ax1 = figs.add_axes(rect_ax1)
K_slider = Slider(
    ax=ax1,
    label = "k",
    valmin = 0.0,
    valmax = 10.0,
    valinit = init_k,
)
ax2 = figs.add_axes(rect_ax2)
H_slider = Slider(
    ax=ax2,
    label = "H",
    valmin = 0.0,
    valmax = 10.0,
    valinit = init_H,
)

def graph_draw(line1,line2,pressure_table,vals,old_vals):
    time.sleep(.01)  # to simulate some work
    if old_vals != vals:
        line1.set_ydata(array_functions_generator(functable,vals[0],vals[1],vals[2],vals[3]))
        line1.set_xdata(pressure_table[vals[2]])
        line2.set_ydata(array_functions_generator(functable,vals[0],vals[1],0,0))
        line1.figure.canvas.draw_idle()
    
    line1.figure.canvas.flush_events()
    old_vals = vals
    return old_vals



axs[0].grid(1)
axs[1].grid(1)



old_vals = [0,0,0,0]


plt.show()
old_vals = 0
async def plot_loot():
    global old_vals
    vals = [H_slider.val,K_slider.val,button_x,button_y]
    old_vals = graph_draw(line1,line2,pressure_table,vals,old_vals)

async def osc_loop():
    """Example main loop that only runs for 10 iterations before finishing"""
    await asyncio.sleep(.0001)


async def main():
    server = AsyncIOOSCUDPServer((ip, port), dispatcher, asyncio.get_event_loop())
    transport, protocol = await server.create_serve_endpoint()  # Create datagram endpoint and start serving
    try:
        while 1:
            await osc_loop()  # Enter main loop of program
            await plot_loot()
    except KeyboardInterrupt:
        print("Closing Program")
        transport.close()  # Clean up serve endpoint

        for synth in group.children[:]:
            synth.free()
        sc_server.quit()

        exit()

        

if sys.version_info >= (3, 7):
    asyncio.run(main())



# for i in range(len(mouth_and_incoming_pressure_difference)-1):
#     equation = mouth_and_incoming_pressure_difference[i]-pressure_difference
#     val = np.intersect1d(flow_vals,equation)
#     print(val)
#     solutions[i] = val



# figs, axs = plt.subplots(1,2)

# axs[0].grid(1)
# line1, = axs[0].plot(pressure_difference,pressure_difference_cal(1,0, pressure_difference),'r-', label = "$p^-_\Delta - p\Delta$")
# line2, = axs[0].plot(pressure_difference,flow_vals,'b--', label = "$u(p\Delta)$")
# axs[0].set_xlabel('Normalized $p\Delta$')
# axs[0].set_ylabel('Flow (u)')
# figs.subplots_adjust(left = 0.25, bottom = 0.15)

# plt.xlim([0,1])
# plt.ylim([0,1])
# plt.legend()
# axI = figs.add_axes([0.125, 0.25, 0.0225, 0.63])
# I_slider = Slider(
#     ax=axI,
#     label = "I",
#     valmin = 0.0,
#     valmax = 10.0,
#     valinit = 0,
#     orientation = "vertical"
# )

# def update(val):
#     line1.set_ydata(pressure_difference_cal(1,I_slider.val, pressure_difference))
#     figs.canvas.draw_idle()
    
# I_slider.on_changed(update)
# axs[0].set_title(label = "Graphical Solution for $p\Delta$")
    
# newly_derived_pd_solution = pressure_difference_solution(mouth_and_incoming_pressure_difference,pressure_difference,init_H,init_k)


# line3 = axs[1].plot(mouth_and_incoming_pressure_difference,newly_derived_pd_solution)
# axs[1].set_xlabel("input pressure difference $[p^-_\Delta - p\Delta]$")
# axs[1].set_ylabel("Pressure Difference Solutions [$p\Delta^'$]")
# axs[1].set_title("$u_d(p\Delta) = p^-_\Delta - p\Delta$")


# plt.show()

# fig1, ax1 = plt.subplots()
# ax1.grid(1)
# line1 = ax1.plot(np.linspace(0,360,100),mouth_and_incoming_pressure_difference)
# ax1.set_xlabel('Angle')
# ax1.set_ylabel('Pressure')
# plt.show()


# pressure = np.linspace(-0.03,1,100)


# initial_exponent_1 = 1/2
# initial_exponent_2 = 3/2

# fig, ax = plt.subplots()
# ax.grid(1)
# line, = ax.plot(pressure, reedFlow_Equation(pressure, init_H, init_k,initial_exponent_1,initial_exponent_2))
# line2, = ax.plot(pressure,linearized_reedFlow_Equation(pressure,init_H,init_k))
# ax.set_xlabel('Angle normalized')
# fig.subplots_adjust(left = 0.35, bottom = 0.35)

# axH = fig.add_axes([0.1, 0.25, 0.0225, 0.63])
# H_slider = Slider(
#     ax=axH,
#     label = "H",
#     valmin = 0.1,
#     valmax = 3,
#     valinit = init_H,
#     orientation = "vertical"
# )
# axK = fig.add_axes([0.2, 0.25, 0.0225, 0.63])
# K_slider = Slider(
#     ax=axK,
#     label = "K",
#     valmin = 0.1,
#     valmax = 3,
#     valinit = init_k,
#     orientation = "vertical"
# )

# axexp1 = fig.add_axes([0.15, 0.05, 0.75, 0.03])
# exp1_slider = Slider(
#     ax=axexp1,
#     label = "$H*\sqrt{p}$",
#     valmin = 0.1,
#     valmax = 3,
#     valinit = initial_exponent_1,
# )
# axexp2 = fig.add_axes([0.15, 0.15, 0.75, 0.03])
# exp2_slider = Slider(
#     ax=axexp2,
#     label = "$p^{3/2}/k$",
#     valmin = 0.1,
#     valmax = 3,
#     valinit = initial_exponent_2
# )

# def update(val):
#     line.set_ydata(reedFlow_Equation(pressure, H_slider.val, K_slider.val,exp1_slider.val,exp2_slider.val))
#     line2.set_ydata(linearized_reedFlow_Equation(pressure,H_slider.val, K_slider.val))
#     fig.canvas.draw_idle()
    
# H_slider.on_changed(update)
# K_slider.on_changed(update)
# exp1_slider.on_changed(update)
# exp2_slider.on_changed(update)

# resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
# button = Button(resetax, 'Reset', hovercolor='0.975')
# plt.title("Reed Flow Equation")
# plt.grid(1)
# def reset(event):
#     H_slider.reset()
#     K_slider.reset()
# button.on_clicked(reset)

# plt.show()



# x = np.linspace(0, 1,3601)
# x_A = np.sqrt(np.abs(x))
# x_B = x*x_A
# x_sgn = np.sign(x)

# H = np.sqrt(27)/2
# k = 27/(H**3*4)


# pressure = np.stack((x,x_A,x_B,x_sgn),axis=1)
# print(H,k)
# print(pressure.shape)
# np.set_printoptions(threshold=10)
# print(pressure)



# def change_button_xp(event):
#     global button_x
#     b_temp = button_x+1
#     button_x = b_temp if button_x < 1 else button_x
#     line1.set_ydata(reedflow_wavetable[button_x,button_y])
#     figs.canvas.draw_idle()
#     return 0

# def change_button_xm(event):
#     global button_x
#     b_temp = button_x-1
#     button_x = b_temp if button_x > 0 else button_x
#     line1.set_ydata(reedflow_wavetable[button_x,button_y])
#     figs.canvas.draw_idle()
#     return 0

# def change_button_yp(event):
#     global button_y
#     b_temp = button_y+1
#     button_y = b_temp if button_y < 1 else button_y
#     line1.set_ydata(reedflow_wavetable[button_x,button_y])
#     figs.canvas.draw_idle()    
#     return 0

# def change_button_ym(event):
#     global button_y
#     b_temp = button_y-1
#     button_y = b_temp if button_y > 0  else button_y
#     line1.set_ydata(reedflow_wavetable[button_x,button_y])
#     figs.canvas.draw_idle()
#     return 0

# x_len = 0.02
# y_len = 0.04
# center_x = 0.50
# disty = 0.05
# distx = 0.03
# center_y = 0.12
# blax = figs.add_axes([center_x-distx, center_y, x_len, y_len])
# brax = figs.add_axes([center_x+distx, center_y, x_len, y_len])
# buax = figs.add_axes([center_x,center_y+disty, x_len, y_len])
# bdax = figs.add_axes([center_x, center_y-disty, x_len, y_len])

# buttonleft = Button(blax, '\u2190', hovercolor='0.975')
# buttonleft.on_clicked(change_button_xm)
# buttonright = Button(brax, '\u2192', hovercolor='0.975')
# buttonright.on_clicked(change_button_xp)
# buttontop = Button(buax, '\u2191', hovercolor='0.975')
# buttontop.on_clicked(change_button_yp)
# buttondown = Button(bdax, '\u2193', hovercolor='0.975')
# buttondown.on_clicked(change_button_ym)