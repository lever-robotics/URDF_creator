import React from 'react';
import DownloadRobotPackage from './components/DownloadRobotPackage/DownloadRobotPackage';
import ThreeScene from './components/URDFviewer/ThreeScene';
import URDFHistoryProvider from './components/URDFContext/URDFHistoryContext';
import URDFGUIProvider from './components/URDFContext/URDFGUIContext';
import URDFCodeProvider from './components/URDFContext/URDFCodeContext';

const App = () => {
  return (
    <URDFHistoryProvider>
      <URDFGUIProvider>
        <ThreeScene />
      </URDFGUIProvider>
      <URDFCodeProvider>
        <DownloadRobotPackage />
      </URDFCodeProvider>
    </URDFHistoryProvider>
  );
};

export default App;
