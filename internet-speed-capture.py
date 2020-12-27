import speedtest
import numpy as np

st = speedtest.Speedtest()
servers = []
st.get_servers(servers)
st.get_best_server()

speed_array = np.array([])

while True:
	try:
		speed = st.download()
		MBPS = speed/1000000.0

		if len(speed_array) < 5:
			speed_array = np.append(speed_array,MBPS)
		else:
			speed_array = np.append(speed_array[1:],MBPS)

		ave_speed = np.average(speed_array)

		print("Download speed {:.4f} Mbps | Average speed {:.4f}".format(MBPS, ave_speed))
	except Exception as e:
		# print(e)
		pass