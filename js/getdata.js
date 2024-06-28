async function fetchData() {
  try {
    const response = await fetch('http://localhost:3000/api/data');
    if (!response.ok) {
      throw new Error('Failed to fetch data');
    }
    const data = await response.json();
    let batteryMur620A, batteryMur620B, batteryMur620C, batteryMur620D = 0;

    // MIR 600A

    if(data.dataA != null){
      stateMur620A = parseInt(data.dataA.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMur620A = parseFloat(data.dataA.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMur620A = parseFloat(data.dataA.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]);
      remainingMur620A = (remainingMur620A / 3600) - 2;
      xMur620A = parseFloat(data.dataA.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMur620A = parseFloat(data.dataA.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMur620A = parseFloat(data.dataA.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      //let errorcount620A = 0;
      errorcount620A = parseFloat(data.dataA.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);

      if(stateMur620D == 4){
        document.getElementById('stateMur620A').textContent = 'GO';
        document.getElementById('stateMur620A').classList.remove('bg-danger');
        document.getElementById('stateMur620A').classList.add('bg-success');
      }else{
        document.getElementById('stateMur620A').textContent = 'STOP';
        document.getElementById('stateMur620A').classList.remove('bg-success');
        document.getElementById('stateMur620A').classList.add('bg-danger');
      }
      

      const batteryElementA = document.getElementById('remainingMur620A');
      if(remainingMur620A < 0){
        batteryElementA.innerHTML = 'REPLACE BATTERY!';
        batteryElementA.classList.add('bg-danger', 'text-white', 'badge');
      }else{
        batteryElementA.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementA.innerHTML = remainingMur620A.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMur620A').textContent = batteryMur620A.toFixed(2) + '%';
      document.getElementById('pos620A').textContent = '[' + xMur620A + 'm; ' + yMur620A + 'm; ' + orientationMur620A + '°]';
      document.getElementById('errorcount620A').textContent = errorcount620A;
      document.getElementById('statusMir600A').classList.remove('bg-danger');
      document.getElementById('statusMir600A').classList.add('bg-success');
      document.getElementById('statusMir600A').textContent = 'MIR: online';
    }else{
      document.getElementById('statusMir600A').textContent = 'MIR: offline';
    }

    if(data.dataMURA != null){
      document.getElementById('statusMUR620A').textContent = 'MUR: online';
      document.getElementById('statusMUR620A').classList.remove('bg-danger');
      document.getElementById('statusMUR620A').classList.add('bg-success');

      pingMUR620A = data.dataMURA.ping;
      document.getElementById('responseTime620A').textContent = pingMUR620A + 'ms';
    }else{
      document.getElementById('statusMUR620A').textContent = 'MUR: offline';
      document.getElementById('statusMUR620A').classList.remove('bg-success');
      document.getElementById('statusMUR620A').classList.add('bg-danger');
    }

    // MIR 600B
    
    if (data.dataB != null) {
      stateMur620B = parseInt(data.dataB.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMur620B = parseFloat(data.dataB.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMur620B = (parseFloat(data.dataB.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]) / 3600) - 2;
      xMur620B = parseFloat(data.dataB.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMur620B = parseFloat(data.dataB.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMur620B = parseFloat(data.dataB.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      errorcount620B = parseFloat(data.dataB.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);
    
      if (stateMur620B == 4) {
        document.getElementById('stateMur620B').textContent = 'GO';
        document.getElementById('stateMur620B').classList.remove('bg-danger');
        document.getElementById('stateMur620B').classList.add('bg-success');
      } else {
        document.getElementById('stateMur620B').textContent = 'STOP';
        document.getElementById('stateMur620B').classList.remove('bg-success');
        document.getElementById('stateMur620B').classList.add('bg-danger');
      }
      
      const batteryElementB = document.getElementById('remainingMur620B');
      if(remainingMur620B < 0){
        batteryElementB.innerHTML = 'REPLACE BATTERY!';
        batteryElementB.classList.add('bg-danger', 'text-white', 'badge');
      }else{
        batteryElementB.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementB.innerHTML = remainingMur620B.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMur620B').textContent = batteryMur620B.toFixed(2) + '%';
      document.getElementById('pos620B').textContent = '[' + xMur620B + 'm; ' + yMur620B + 'm; ' + orientationMur620B + '°]';
      document.getElementById('errorcount620B').textContent = errorcount620B;
      document.getElementById('statusMir600B').classList.remove('bg-danger');
      document.getElementById('statusMir600B').classList.add('bg-success');
      document.getElementById('statusMir600B').textContent = 'MIR: online';
    } else {
      document.getElementById('statusMir600B').textContent = 'MIR: offline';
    }

    if(data.dataMURB != null){
      document.getElementById('statusMUR620B').textContent = 'MUR: online';
      document.getElementById('statusMUR620B').classList.remove('bg-danger');
      document.getElementById('statusMUR620B').classList.add('bg-success');

      pingMUR620B = data.dataMURB.ping;
      document.getElementById('responseTime620B').textContent = pingMUR620B + 'ms';
    }else{
      document.getElementById('statusMUR620B').textContent = 'MUR: offline';
      document.getElementById('statusMUR620B').classList.remove('bg-success');
      document.getElementById('statusMUR620B').classList.add('bg-danger');
    }

    // MIR 600C

    if (data.dataC != null) {
      stateMur620C = parseInt(data.dataC.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMur620C = parseFloat(data.dataC.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMur620C = (parseFloat(data.dataC.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]) / 3600) - 2;
      xMur620C = parseFloat(data.dataC.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMur620C = parseFloat(data.dataC.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMur620C = parseFloat(data.dataC.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      errorcount620C = parseFloat(data.dataC.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);
    
      if (stateMur620C == 4) {
        document.getElementById('stateMur620C').textContent = 'GO';
        document.getElementById('stateMur620C').classList.remove('bg-danger');
        document.getElementById('stateMur620C').classList.add('bg-success');
      } else {
        document.getElementById('stateMur620C').textContent = 'STOP';
        document.getElementById('stateMur620C').classList.remove('bg-success');
        document.getElementById('stateMur620C').classList.add('bg-danger');
      }
      
      const batteryElementC = document.getElementById('remainingMur620C');
      if(remainingMur620C < 0){
        batteryElementC.innerHTML = 'REPLACE BATTERY!';
        batteryElementC.classList.add('bg-danger', 'text-white', 'badge');
      }else{
        batteryElementC.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementC.innerHTML = remainingMur620C.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMur620C').textContent = batteryMur620C.toFixed(2) + '%';
      document.getElementById('pos620C').textContent = '[' + xMur620C + 'm; ' + yMur620C + 'm; ' + orientationMur620C + '°]';
      document.getElementById('errorcount620C').textContent = errorcount620C;
      document.getElementById('statusMir600C').classList.remove('bg-danger');
      document.getElementById('statusMir600C').classList.add('bg-success');
      document.getElementById('statusMir600C').textContent = 'MIR: online';
      
    } else {
      document.getElementById('statusMir600C').textContent = 'MIR: offline';
    }

    if(data.dataMURC != null){
      document.getElementById('statusMUR620C').textContent = 'MUR: online';
      document.getElementById('statusMUR620C').classList.remove('bg-danger');
      document.getElementById('statusMUR620C').classList.add('bg-success');

      pingMUR620C = data.dataMURC.ping;
      document.getElementById('responseTime620C').textContent = pingMUR620C + 'ms';
    } else {
      document.getElementById('statusMUR620C').textContent = 'MUR: offline';
      document.getElementById('statusMUR620C').classList.remove('bg-success');
      document.getElementById('statusMUR620C').classList.add('bg-danger');
    }
    
    // MIR 600D

    if(data.dataD != null){
      stateMur620D = parseInt(data.dataD.match(/mir_robot_state_id ([\d.]+)/)[1]);
      batteryMur620D = parseFloat(data.dataD.match(/mir_robot_battery_percent ([\d.]+)/)[1]);
      remainingMur620D = parseFloat(data.dataD.match(/mir_robot_battery_time_remaining_seconds ([\d.]+)/)[1]);
      remainingMur620D = (remainingMur620D / 3600) - 2;
      xMur620D = parseFloat(data.dataD.match(/mir_robot_position_x_meters ([\d.]+)/)[1]).toFixed(4);
      yMur620D = parseFloat(data.dataD.match(/mir_robot_position_y_meters ([\d.]+)/)[1]).toFixed(4);
      orientationMur620D = parseFloat(data.dataD.match(/mir_robot_orientation_degrees\s+(-?[\d.]+)/)[1]).toFixed(4);
      errorcount620D = parseFloat(data.dataD.match(/mir_robot_errors ([\d.]+)/)[1]).toFixed(0);

      console.log(stateMur620D);
      if(stateMur620D == 4){
        document.getElementById('stateMur620D').textContent = 'GO';
        document.getElementById('stateMur620D').classList.remove('bg-danger');
        document.getElementById('stateMur620D').classList.add('bg-success');
      }else{
        document.getElementById('stateMur620D').textContent = 'STOP';
        document.getElementById('stateMur620D').classList.remove('bg-success');
        document.getElementById('stateMur620D').classList.add('bg-danger');
      }
      
      const batteryElementD = document.getElementById('remainingMur620D');
      if(remainingMur620D < 0){
        batteryElementD.innerHTML = 'REPLACE BATTERY!';
        batteryElementD.classList.add('bg-danger', 'text-white', 'badge');
      }else{
        batteryElementD.classList.remove('bg-danger', 'text-white', 'badge');
        batteryElementD.innerHTML = remainingMur620D.toFixed(2) + 'h';
      }

      document.getElementById('batteryLevelMur620D').textContent = batteryMur620D.toFixed(2) + '%';
      document.getElementById('pos620D').textContent = '[' + xMur620D+ 'm; ' + yMur620D + 'm; ' + orientationMur620D + '°]';
      document.getElementById('errorcount620D').textContent = errorcount620D;
      document.getElementById('statusMir600D').classList.remove('bg-danger');
      document.getElementById('statusMir600D').classList.add('bg-success');
      document.getElementById('statusMir600D').textContent = 'MIR: online';
    }else{
      document.getElementById('statusMir600D').textContent = 'MIR: offline';
    }

    if(data.dataMURD != null){
      document.getElementById('statusMUR620D').textContent = 'MUR: online';
      document.getElementById('statusMUR620D').classList.remove('bg-danger');
      document.getElementById('statusMUR620D').classList.add('bg-success');

      pingMUR620D = data.dataMURD.ping;
      battery620D = data.dataMURD.battery;
      document.getElementById('responseTime620D').textContent = pingMUR620D + 'ms';
      document.getElementById('batteryMUR620D').textContent = battery620D + '%';
    }else{
      document.getElementById('statusMUR620D').textContent = 'MUR: offline';
      document.getElementById('statusMUR620D').classList.remove('bg-success');
      document.getElementById('statusMUR620D').classList.add('bg-danger');
    }


  } catch (error) {
    console.error('Error fetching data:', error);
  }

}


fetchData()