import React, { StrictMode, useEffect } from 'react';
import ThreeDisplay from './components/ThreeDisplay/ThreeDisplay';
import initializeAnalytics from './analytics';

const App = () => {
  // initialize Google Analytics
  useEffect(() => {
    initializeAnalytics();
  }, []);


  return (
    // Taking strictmode off makes the scaling of meshes really jerky
    <StrictMode>
      <ThreeDisplay/>
    </StrictMode>
  );
};

export default App;
