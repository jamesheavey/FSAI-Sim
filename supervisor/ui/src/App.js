import React, { useState, useEffect, createContext, useReducer } from "react";
import { Helmet } from "react-helmet";
import Dashboard from "./Dashboard";
import useRos from "./useRos"



export const Context = createContext();

function App() {
  const context = useRos();

  return (
    <div className="App">
      <Helmet></Helmet>
      <Context.Provider value={context}>
        <Dashboard />
      </Context.Provider>
    </div>
  );
}

export default App;
