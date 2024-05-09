import React from 'react';
import { URDFProvider } from './components/URDFContext/URDFContext';
import URDFViewer from './components/URDFviewer/URDFViewer';

function App() {
  return (
    <URDFProvider>
      <div style={{ display: 'flex', flexDirection: 'row', height: '100vh', width: '100vw' }}>
        <URDFViewer />
        {/* You can add more components here if necessary */}
      </div>
    </URDFProvider>
  );
}

export default App;
