// src/analytics.js
// Used to initialize Google Analytics
import ReactGA from 'react-ga4';

const initializeAnalytics = () => {
  ReactGA.initialize('G-MD47RTJTDL'); // Replace with your Measurement ID
};

export default initializeAnalytics;