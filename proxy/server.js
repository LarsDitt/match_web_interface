// ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA

const express = require('express');
const axios = require('axios');
const cors = require('cors');

const app = express();
const port = 3000;

// Enable CORS for all requests
app.use(cors());

let combinedData = {
    dataA: null,
    dataB: null,
    dataC: null,
    dataD: null,
    dataMURC: null
};

const fetchData = async () => {
    console.log("Fetching data...")
    let dataA, dataB, dataC, dataD = null;
    let dataMURA, dataMURB, dataMURC, dataMURD = null;
    const urlA = 'http://10.145.8.61/api/v2.0.0/metrics';
    const urlB = 'http://10.145.8.49/api/v2.0.0/metrics';
    const urlC = 'http://10.145.8.48/api/v2.0.0/metrics';
    const urlD = 'http://10.145.8.44/api/v2.0.0/metrics';
    const urlMURA = 'http://mur620a:5678/metrics'
    const urlMURB = 'http://mur620b:5678/metrics'
    const urlMURC = 'http://mur620c:5678/metrics'
    const urlMURD = 'http://mur620d:5678/metrics'

    const headers = {
        'accept': 'text/plain',
        'Authorization': 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==',
        'Accept-Language': 'en_US'
    };

    // Add the timeout to the axios config
    const axiosConfig = {
        headers: headers,
        timeout: 250
    };

    const axiosConfig_MUR = {
        timeout: 250
    };

    try {
        const responseA = await axios.get(urlA, axiosConfig);
        dataA = responseA.data;
    } catch (error) {
        console.log("Offline: MIRA");
        dataA = null;
    }

    try{
        const responseB = await axios.get(urlB, axiosConfig);
        dataB = responseB.data;
    } catch (error) {
        console.log("Offline: MIRB");
        dataB = null;
    }

    try{
        const responseC = await axios.get(urlC, axiosConfig);
        dataC = responseC.data;
    } catch (error) {
        console.log("Offline: MIRC");
        dataC = null;
    }
    
    try {
        const responseD = await axios.get(urlD, axiosConfig);
        dataD = responseD.data;
    } catch (error) {
        console.log("Offline: MIRD");
        dataD = null;
    }

    try {
        const responseMURA = await axios.get(urlMURA, axiosConfig_MUR); // Fetch data from the Python Flask server
        console.log(responseMURA.data);
        dataMURA = responseMURA.data;
    } catch (error) {
        console.log("Offline: MURA");
    }

    try {
        const responseMURB = await axios.get(urlMURB, axiosConfig_MUR); // Fetch data from the Python Flask server
        console.log(responseMURB.data);
        dataMURB = responseMURB.data;
    } catch (error) {
        console.log("Offline: MURB");
    }

    try {
        const responseMURC = await axios.get(urlMURC, axiosConfig_MUR); // Fetch data from the Python Flask server
        console.log(responseMURC.data);
        dataMURC = responseMURC.data;
    } catch (error) {
        console.log("Offline: MURC");
    }

    try {
        const responseMURD = await axios.get(urlMURD, axiosConfig_MUR); // Fetch data from the Python Flask server
        console.log(responseMURD.data);
        dataMURD = responseMURD.data;
    } catch (error) {
        console.log("Offline: MURD");
    }

    combinedData = {
        dataA: dataA,
        dataB: dataB,
        dataC: dataC,
        dataD: dataD,
        dataMURA: dataMURA,
        dataMURB: dataMURB,
        dataMURC: dataMURC,
        dataMURD: dataMURD
    };
}

setInterval(fetchData, 2000);

// Define the route to fetch data from the API
app.get('/api/data', async (req, res) => {
    res.json(combinedData); // Send combined JSON response back to the client
});

// Start the server
app.listen(port, () => {
  console.log(`Server is running on http://localhost:${port}`);
});
