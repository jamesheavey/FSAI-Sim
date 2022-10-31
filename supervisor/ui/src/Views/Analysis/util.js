export const getSelectedDataOfType = (report, key) =>
  report?.selectedDatapoints
    ?.filter((p) => p.key == key)
    ?.map((p) => ({ ...p.data, location: p.location })) || [];

export const getDataOfType = (report, key) =>
  report?.datapoints
    ?.filter((p) => p.key == key)
    ?.map((p) => ({ ...p.data, location: p.location })) || [];

export const filterData = (report, filter = () => true) =>
  report?.datapoints
    ?.filter(filter)
    ?.map((p) => ({ ...p.data, key: p.key, location: p.location })) || [];

export const filterSelectedData = (report, filter = () => true) =>
  report?.selectedDatapoints
    ?.filter(filter)
    ?.map((p) => ({ ...p.data, key: p.key, location: p.location })) || [];
