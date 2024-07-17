document.addEventListener('DOMContentLoaded', function () {
  const autoReloadSwitch = document.getElementById('autoReloadSwitch');

  async function fetchData() {
    try {
      const response = await fetch('http://10.145.8.39:3000/api/data');
      if (!response.ok) {
        throw new Error('Failed to fetch data');
      }
      let data = await response.json();

      const robots = ['A', 'B', 'C', 'D'];

      robots.forEach(robot => {
        handleMIRData(data[`data${robot}`], robot);
        handleMURData(data[`dataMUR${robot}`], robot);
      });

    } catch (error) {
      console.error('Error fetching data:', error);
    }
    data = null;
  }

  function handleMIRData(mirData, robot) {
    if (mirData != null) {
      const state = parseInt(mirData.match(/mir_robot_state_id ([\d.]+)/)[1]);
      const battery = parseFloat(mirData.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      const remaining = (parseFloat(mirData.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]) / 3600) - 2;
      const x = parseFloat(mirData.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      const y = parseFloat(mirData.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      const orientation = parseFloat(mirData.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      let errorcount = parseFloat(mirData.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);

      const stateElement = document.getElementById(`stateMIR600${robot}`);
      if (state == 4) {
        stateElement.textContent = 'GO';
        stateElement.classList.remove('bg-danger');
        stateElement.classList.add('bg-success');
      } else {
        stateElement.textContent = 'STOP';
        stateElement.classList.remove('bg-success');
        stateElement.classList.add('bg-danger');
      }

      const batteryElement = document.getElementById(`remainingMIR600${robot}`);
      if (remaining < 0) {
        batteryElement.innerHTML = 'REPLACE BATTERY!';
        batteryElement.classList.add('bg-danger', 'text-white', 'badge');
      } else {
        batteryElement.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElement.innerHTML = remaining.toFixed(2) + 'h';
      }

      document.getElementById(`batteryLevelMIR600${robot}`).textContent = battery.toFixed(2) + '%';
      document.getElementById(`posMIR600${robot}`).textContent = `[${x}m; ${y}m; ${orientation}°]`;
      const errorcountMIR = document.getElementById(`errorcountMIR600${robot}`);
      errorcountMIR.textContent = errorcount;
      if(errorcount > 0) {
        errorcountMIR.classList.add('bg-danger', 'badge', 'text-white');
      }else{
        errorcountMIR.classList.remove('bg-danger');
      }

      const statusElement = document.getElementById(`statusMIR600${robot}`);
      statusElement.classList.remove('bg-danger');
      statusElement.classList.add('bg-success');
      statusElement.textContent = 'MIR: online';
    } else {
      document.getElementById(`statusMIR600${robot}`).textContent = 'MIR: offline';
    }
  }

  function handleMURData(murData, robot) {
    const statusElement = document.getElementById(`statusMUR620${robot}`);
    if (murData != null) {
      statusElement.textContent = 'MUR: online';
      statusElement.classList.remove('bg-danger');
      statusElement.classList.add('bg-success');

      const ping = murData.ping;
      const battery = murData.battery.toFixed(2);
      document.getElementById(`responseTimeMUR620${robot}`).textContent = ping + 'ms';
      document.getElementById(`batteryMUR620${robot}`).textContent = battery + '%';

      if (murData.URL !== undefined && murData.URR !== undefined) {
        updateUR(robot, 'URL', murData.URL);
        updateUR(robot, 'URR', murData.URR);
      }
    } else {
      statusElement.textContent = 'MUR: offline';
      statusElement.classList.remove('bg-success');
      statusElement.classList.add('bg-danger');
      document.getElementById(`responseTimeMUR620${robot}`).textContent = '-';
      document.getElementById(`batteryMUR620${robot}`).textContent = '-';
      updateUR(robot, 'URL', "OFFLINE");
      updateUR(robot, 'URR', "OFFLINE");
    }
  }

  function updateUR(robot, type, status) {
    const element = document.getElementById(`statusMUR620${robot}_${type}`);
    if (!element) return;

    element.classList.remove('bg-secondary', 'bg-success', 'bg-warning', 'bg-danger');

    switch (status) {
      case "True/False/True":
        element.textContent = "GO";
        element.classList.add('bg-success');
        break;
      case "True/False/False":
        element.textContent = "PROG NOT RUNNING";
        element.classList.add('bg-warning');
        break;
      case "False/False/False":
        element.textContent = "STOP";
        element.classList.add('bg-danger');
        break;
      case "OFFLINE":
      default:
        element.textContent = "OFFLINE";
        element.classList.add('bg-secondary');
    }
  }
  async function startFetching() {
    // Fetch data immediately on page load
    await fetchData();

    while (true) {
      await new Promise(r => setTimeout(r, 2000));

      // Check if the switch is enabled
      if (autoReloadSwitch.checked) {
        await fetchData();
      }
    }
  }

  startFetching();
});