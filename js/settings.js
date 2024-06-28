// Function to save settings
function saveSettings() {
    const rosServerIpInput = document.getElementById('ros_server_ip').value;
    localStorage.setItem('rosServerIp', rosServerIpInput); // Save to localStorage
    console.log('Settings saved:', { rosServerIp: rosServerIpInput });
    alert('Settings saved successfully!');
}

// Function to load settings from localStorage
function loadSettings() {
    const rosServerIpInput = document.getElementById('ros_server_ip');
    if (rosServerIpInput) {
        const savedRosServerIp = localStorage.getItem('rosServerIp');
        if (savedRosServerIp !== null) {
            rosServerIpInput.value = savedRosServerIp;
            console.log('Settings loaded:', { rosServerIp: savedRosServerIp });
        } else {
            rosServerIpInput.value = 'ws://localhost:9090'; // Default value
        }
    } else {
        console.error('Element with ID "ros_server_ip" not found.');
    }
}