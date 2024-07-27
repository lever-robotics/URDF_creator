import React, { StrictMode } from 'react';
import SceneState from './components/SceneState';

const App = () => {
  return (
    // Taking strictmode off makes the scaling of meshes really jerky
    <StrictMode>
      <SceneState />
    </StrictMode>
  );
};

export default App;
