async function fetchData() {
  try {
    const response = await fetch('http://localhost:3000/api/data');
    if (!response.ok) {
      throw new Error('Failed to fetch data');
    }
    const data = await response.json();
    let batteryMUR620A, batteryMUR620B, batteryMUR620C, batteryMUR620D = 0;

    // MIR 600A

    if(data.dataA != null){
      stateMIR600A = parseInt(data.dataA.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMIR600A= parseFloat(data.dataA.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMIR600A = parseFloat(data.dataA.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]);
      remainingMIR600A = (remainingMIR600A / 3600) - 2;
      xMurMIR600A = parseFloat(data.dataA.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMurMIR600A = parseFloat(data.dataA.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMIR600A = parseFloat(data.dataA.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      //let errorcount620A = 0;
      errorcountMIR600A = parseFloat(data.dataA.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);

      if(stateMIR600A == 4){
        document.getElementById('stateMIR600A').textContent = 'GO';
        document.getElementById('stateMIR600A').classList.remove('bg-danger');
        document.getElementById('stateMIR600A').classList.add('bg-success');
      }else{
        document.getElementById('stateMIR600A').textContent = 'STOP';
        document.getElementById('stateMIR600A').classList.remove('bg-success');
        document.getElementById('stateMIR600A').classList.add('bg-danger');
      }
      

      const batteryElementA = document.getElementById('remainingMIR600A');
      if(remainingMIR600A < 0){
        batteryElementA.innerHTML = 'REPLACE BATTERY!';
        batteryElementA.classList.add('bg-danger', 'text-white', 'badge');
      }else{
        batteryElementA.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementA.innerHTML = remainingMIR600A.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMIR600A').textContent = batteryMIR600A.toFixed(2) + '%';
      document.getElementById('posMIR600A').textContent = '[' + xMurMIR600A + 'm; ' + yMurMIR600A + 'm; ' + orientationMIR600A + '°]';
      document.getElementById('errorcountMIR600A').textContent = errorcountMIR600A;
      document.getElementById('statusMIR600A').classList.remove('bg-danger');
      document.getElementById('statusMIR600A').classList.add('bg-success');
      document.getElementById('statusMIR600A').textContent = 'MIR: online';
    }else{
      document.getElementById('statusMIR600A').textContent = 'MIR: offline';
    }

    if(data.dataMURA != null){
      document.getElementById('statusMUR620A').textContent = 'MUR: online';
      document.getElementById('statusMUR620A').classList.remove('bg-danger');
      document.getElementById('statusMUR620A').classList.add('bg-success');

      pingMUR620A = data.dataMURA.ping;
      batteryMUR620A = data.dataMURA.battery.toFixed(2);
      document.getElementById('responseTimeMUR620A').textContent = pingMUR620A + 'ms';
      document.getElementById('batteryMUR620A').textContent = batteryMUR620A + '%';
    }else{
      document.getElementById('statusMUR620A').textContent = 'MUR: offline';
      document.getElementById('statusMUR620A').classList.remove('bg-success');
      document.getElementById('statusMUR620A').classList.add('bg-danger');
    }


    // MIR 600B

    if (data.dataB != null) {
      stateMIR600B = parseInt(data.dataB.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMIR600B = parseFloat(data.dataB.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMIR600B = (parseFloat(data.dataB.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]) / 3600) - 2;
      xMIR600B = parseFloat(data.dataB.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMIR600B = parseFloat(data.dataB.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMIR600B = parseFloat(data.dataB.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      errorcountMIR600B = parseFloat(data.dataB.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);

      if (stateMIR600B == 4) {
        document.getElementById('stateMIR600B').textContent = 'GO';
        document.getElementById('stateMIR600B').classList.remove('bg-danger');
        document.getElementById('stateMIR600B').classList.add('bg-success');
      } else {
        document.getElementById('stateMIR600B').textContent = 'STOP';
        document.getElementById('stateMIR600B').classList.remove('bg-success');
        document.getElementById('stateMIR600B').classList.add('bg-danger');
      }
      
      const batteryElementB = document.getElementById('remainingMIR600B');
      if(remainingMIR600B < 0){
        batteryElementB.innerHTML = 'REPLACE BATTERY!';
        batteryElementB.classList.add('bg-danger', 'text-white', 'badge');
      }else{
        batteryElementB.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementB.innerHTML = remainingMIR600B.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMIR600B').textContent = batteryMIR600B.toFixed(2) + '%';
      document.getElementById('posMIR600B').textContent = '[' + xMIR600B + 'm; ' + yMIR600B + 'm; ' + orientationMIR600B + '°]';
      document.getElementById('errorcountMIR600B').textContent = errorcountMIR600B;
      document.getElementById('statusMIR600B').classList.remove('bg-danger');
      document.getElementById('statusMIR600B').classList.add('bg-success');
      document.getElementById('statusMIR600B').textContent = 'MIR: online';
    } else {
      document.getElementById('statusMIR600B').textContent = 'MIR: offline';
    }

    if(data.dataMURB != null){
      document.getElementById('statusMUR620B').textContent = 'MUR: online';
      document.getElementById('statusMUR620B').classList.remove('bg-danger');
      document.getElementById('statusMUR620B').classList.add('bg-success');

      pingMUR620B = data.dataMURB.ping;
      batteryMUR620B = data.dataMURB.battery.toFixed(2);
      document.getElementById('responseTimeMUR620B').textContent = pingMUR620B + 'ms';
      document.getElementById('batteryMUR620B').textContent = batteryMUR620B + '%';
    }else{
      document.getElementById('statusMUR620B').textContent = 'MUR: offline';
      document.getElementById('statusMUR620B').classList.remove('bg-success');
      document.getElementById('statusMUR620B').classList.add('bg-danger');
    }

    // MIR 600C

    if (data.dataC != null) {
      stateMIR600C = parseInt(data.dataC.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMIR600C = parseFloat(data.dataC.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMIR600C = parseFloat(data.dataC.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]);
      remainingMIR600C = (remainingMIR600C / 3600) - 2;
      xMIR600C = parseFloat(data.dataC.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMIR600C = parseFloat(data.dataC.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMIR600C = parseFloat(data.dataC.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      errorcountMIR600C = parseFloat(data.dataC.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);

      if (stateMIR600C == 4) {
        document.getElementById('stateMIR600C').textContent = 'GO';
        document.getElementById('stateMIR600C').classList.remove('bg-danger');
        document.getElementById('stateMIR600C').classList.add('bg-success');
      } else {
        document.getElementById('stateMIR600C').textContent = 'STOP';
        document.getElementById('stateMIR600C').classList.remove('bg-success');
        document.getElementById('stateMIR600C').classList.add('bg-danger');
      }
      
      const batteryElementC = document.getElementById('remainingMIR600C');
      if (remainingMIR600C < 0) {
        batteryElementC.innerHTML = 'REPLACE BATTERY!';
        batteryElementC.classList.add('bg-danger', 'text-white', 'badge');
      } else {
        batteryElementC.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementC.innerHTML = remainingMIR600C.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMIR600C').textContent = batteryMIR600C.toFixed(2) + '%';
      document.getElementById('posMIR600C').textContent = '[' + xMIR600C + 'm; ' + yMIR600C + 'm; ' + orientationMIR600C + '°]';
      document.getElementById('errorcountMIR600C').textContent = errorcountMIR600C;
      document.getElementById('statusMIR600C').classList.remove('bg-danger');
      document.getElementById('statusMIR600C').classList.add('bg-success');
      document.getElementById('statusMIR600C').textContent = 'MIR: online';
    } else {
      document.getElementById('statusMIR600C').textContent = 'MIR: offline';
    }

    if (data.dataMURC != null) {
      document.getElementById('statusMUR620C').textContent = 'MUR: online';
      document.getElementById('statusMUR620C').classList.remove('bg-danger');
      document.getElementById('statusMUR620C').classList.add('bg-success');

      pingMUR620C = data.dataMURC.ping;
      batteryMUR620C = data.dataMURC.battery.toFixed(2);
      document.getElementById('responseTimeMUR620C').textContent = pingMUR620C + 'ms';
      document.getElementById('batteryMUR620C').textContent = batteryMUR620C + '%';
    } else {
      document.getElementById('statusMUR620C').textContent = 'MUR: offline';
      document.getElementById('statusMUR620C').classList.remove('bg-success');
      document.getElementById('statusMUR620C').classList.add('bg-danger');
    }
    
    // MIR 600D

    if (data.dataD != null) {
      stateMIR600D = parseInt(data.dataD.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMIR600D = parseFloat(data.dataD.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMIR600D = (parseFloat(data.dataD.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]) / 3600) - 2;
      xMIR600D = parseFloat(data.dataD.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMIR600D = parseFloat(data.dataD.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMIR600D = parseFloat(data.dataD.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      errorcountMIR600D = parseFloat(data.dataD.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);

      if (stateMIR600D == 4) {
        document.getElementById('stateMIR600D').textContent = 'GO';
        document.getElementById('stateMIR600D').classList.remove('bg-danger');
        document.getElementById('stateMIR600D').classList.add('bg-success');
      } else {
        document.getElementById('stateMIR600D').textContent = 'STOP';
        document.getElementById('stateMIR600D').classList.remove('bg-success');
        document.getElementById('stateMIR600D').classList.add('bg-danger');
      }
      
      const batteryElementD = document.getElementById('remainingMIR600D');
      if (remainingMIR600D < 0) {
        batteryElementD.innerHTML = 'REPLACE BATTERY!';
        batteryElementD.classList.add('bg-danger', 'text-white', 'badge');
      } else {
        batteryElementD.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementD.innerHTML = remainingMIR600D.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMIR600D').textContent = batteryMIR600D.toFixed(2) + '%';
      document.getElementById('posMIR600D').textContent = '[' + xMIR600D + 'm; ' + yMIR600D + 'm; ' + orientationMIR600D + '°]';
      document.getElementById('errorcountMIR600D').textContent = errorcountMIR600D;
      document.getElementById('statusMIR600D').classList.remove('bg-danger');
      document.getElementById('statusMIR600D').classList.add('bg-success');
      document.getElementById('statusMIR600D').textContent = 'MIR: online';
    } else {
      document.getElementById('statusMIR600D').textContent = 'MIR: offline';
    }

    if (data.dataMURD != null) {
      document.getElementById('statusMUR620D').textContent = 'MUR: online';
      document.getElementById('statusMUR620D').classList.remove('bg-danger');
      document.getElementById('statusMUR620D').classList.add('bg-success');

      pingMUR620D = data.dataMURD.ping;
      batteryMUR620D = data.dataMURD.battery.toFixed(2);
      document.getElementById('responseTimeMUR620D').textContent = pingMUR620D + 'ms';
      document.getElementById('batteryMUR620D').textContent = batteryMUR620D + '%';
    } else {
      document.getElementById('statusMUR620D').textContent = 'MUR: offline';
      document.getElementById('statusMUR620D').classList.remove('bg-success');
      document.getElementById('statusMUR620D').classList.add('bg-danger');
    }


  } catch (error) {
    console.error('Error fetching data:', error);
  }

}


fetchData()