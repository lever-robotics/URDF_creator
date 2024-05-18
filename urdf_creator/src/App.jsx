import React from 'react';
import ThreeScene from './components/ThreeScene/ThreeScene';
import URDFCodeDisplayer from './components/CodeDisplayer/URDFCodeDisplayer';
import { StateProvider } from './components/URDFContext/StateContext';

const App = () => {
  return (
    <StateProvider>
      <div className='screen'>
        <ThreeScene />
        <URDFCodeDisplayer />
      </div>
    </StateProvider>
  );
};

export default App;
