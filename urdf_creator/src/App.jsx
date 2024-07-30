import React, { StrictMode } from 'react';
import ThreeDisplay from './components/ThreeDisplay/ThreeDisplay';

const App = () => {
  return (
    // Taking strictmode off makes the scaling of meshes really jerky
    <StrictMode>
      <ThreeDisplay/>
    </StrictMode>
  );
};

export default App;
